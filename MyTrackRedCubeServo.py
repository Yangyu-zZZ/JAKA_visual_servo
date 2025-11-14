import time
import sys
import traceback
import threading
import queue
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Tuple, List
from pathlib import Path

# 第三方依赖
try:
    import pyrealsense2 as rs
    import numpy as np
    import cv2
    import serial
    import msvcrt
except ImportError as e:
    print(f"❌ 缺少必要模块: {e}")
    print("请安装: pip install pyrealsense2 numpy opencv-python pyserial")
    sys.exit(1)

# JAKA机器人模块
try:
    import __common
    from jaka_robot import JAKARobotController
    from jaka_robot.constants import RobotConstants
except ImportError as e:
    print(f"❌ 无法导入 jaka_robot: {e}")
    sys.exit(1)


# ============================================================================
# 配置管理
# ============================================================================

@dataclass
class RobotConfig:
    """机器人配置"""
    ip: str = "10.5.5.100"
    home_joint_deg: List[float] = field(default_factory=lambda: [0, 0, 90, 0, 90, -90])
    # plane_z: float = 250.0  # 工作平面高度(mm)
    
    @property
    def home_joint_rad(self) -> List[float]:
        return [np.deg2rad(j) for j in self.home_joint_deg]


# @dataclass
# class GripperConfig:
#     """夹爪配置"""
#     port: str = "COM6"
#     baudrate: int = 115200
#     timeout: float = 1.0


@dataclass
class CameraConfig:
    """相机配置"""
    width: int = 640
    height: int = 480
    fps: int = 60
    format: str = "bgr8"


@dataclass
class ControlConfig:
    """控制参数配置"""
    # 像素到毫米的转换增益
    gain_x_per_pixel: float = 0.05
    gain_y_per_pixel: float = 0.05
    
    # PID参数
    use_pid: bool = True
    pid_x_kp: float = 0.05
    pid_x_ki: float = 0.0
    pid_x_kd: float = 0.01
    pid_y_kp: float = 0.05
    pid_y_ki: float = 0.0
    pid_y_kd: float = 0.01
    pid_integral_limit: float = 500.0
    
    # 限制参数
    pixel_dead_band: float = 3.0
    stable_pixel_band: float = 2.0
    stable_required_frames: int = 5
    max_step_mm: float = 20.0
    min_cmd_mm: float = 0.2
    
    # 平滑参数
    use_ema_smooth: bool = True
    ema_alpha: float = 0.3
    
    # 伺服参数
    use_servo_streaming: bool = True
    servo_interval: float = 0.0167  # ~60Hz
    
    # 滤波器配置
    use_servo_filter: bool = True
    servo_filter_type: str = 'CARTESIAN_NLF'
    cart_max_vp: float = 180.0
    cart_max_ap: float = 800.0
    cart_max_jp: float = 4000.0
    cart_max_vr: float = 60.0
    cart_max_ar: float = 300.0
    cart_max_jr: float = 1500.0


@dataclass
class VisionConfig:
    """视觉检测配置"""
    # HSV红色阈值范围
    red_lower_1: Tuple[int, int, int] = (0, 70, 50)
    red_upper_1: Tuple[int, int, int] = (10, 255, 255)
    red_lower_2: Tuple[int, int, int] = (170, 70, 50)
    red_upper_2: Tuple[int, int, int] = (180, 255, 255)
    
    # 形态学操作
    morph_kernel_size: int = 5
    min_contour_area: float = 200.0
    approx_epsilon: float = 0.02
    
    # 调试
    debug_save_interval: int = 0  # 0=不保存


# ============================================================================
# 运行状态
# ============================================================================

class SystemState(Enum):
    """系统运行状态"""
    IDLE = "idle"
    CAPTURING = "capturing"
    TRACKING = "tracking"
    PAUSED = "paused"
    ERROR = "error"


# ============================================================================
# PID控制器
# ============================================================================

class PIDController:
    """PID控制器实现"""
    
    def __init__(self, kp: float, ki: float, kd: float, 
                 output_limit: float, integral_limit: float):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.output_limit = float(output_limit)
        self.integral_limit = float(integral_limit)
        
        self.integral = 0.0
        self.prev_error: Optional[float] = None
    
    def reset(self) -> None:
        """重置PID状态"""
        self.integral = 0.0
        self.prev_error = None
    
    def update(self, error: float, dt: float) -> float:
        """
        计算PID输出
        
        Args:
            error: 当前误差
            dt: 时间步长(秒)
            
        Returns:
            控制输出值
        """
        dt = max(dt, 1e-3)  # 防止除零
        
        # 积分项
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)
        
        # 微分项
        if self.prev_error is None:
            derivative = 0.0
        else:
            derivative = (error - self.prev_error) / dt
        self.prev_error = error
        
        # PID输出
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = np.clip(output, -self.output_limit, self.output_limit)
        
        return float(output)


# ============================================================================
# 硬件接口层
# ============================================================================

# class Gripper:
#     """夹爪控制器"""
    
#     # 命令字节序列
#     CMD_OPEN = bytes([0x7b, 0x01, 0x02, 0x00, 0x20, 0x49, 0x20, 0x00, 0xc8, 0xF9, 0x7d])
#     CMD_CLOSE = bytes([0x7b, 0x01, 0x02, 0x01, 0x20, 0x49, 0x20, 0x00, 0xc8, 0xF8, 0x7d])
    
#     def __init__(self, config: GripperConfig):
#         self.config = config
#         self.serial: Optional[serial.Serial] = None
    
#     def initialize(self) -> bool:
#         """初始化串口连接"""
#         try:
#             self.serial = serial.Serial(
#                 port=self.config.port,
#                 baudrate=self.config.baudrate,
#                 timeout=self.config.timeout
#             )
#             if not self.serial.is_open:
#                 print(f"❌ 串口 {self.config.port} 打开失败")
#                 return False
#             print(f"✅ 夹爪串口 {self.config.port} 已连接")
#             return True
#         except Exception as e:
#             print(f"❌ 夹爪初始化失败: {e}")
#             return False
    
#     def open(self) -> bool:
#         """打开夹爪"""
#         if self.serial and self.serial.is_open:
#             try:
#                 self.serial.write(self.CMD_OPEN)
#                 print("🔓 夹爪打开")
#                 return True
#             except Exception as e:
#                 print(f"❌ 夹爪打开失败: {e}")
#                 return False
#         return False
    
#     def close(self) -> bool:
#         """闭合夹爪"""
#         if self.serial and self.serial.is_open:
#             try:
#                 self.serial.write(self.CMD_CLOSE)
#                 print("🔒 夹爪闭合")
#                 return True
#             except Exception as e:
#                 print(f"❌ 夹爪闭合失败: {e}")
#                 return False
#         return False
    
#     def cleanup(self) -> None:
#         """清理资源"""
#         if self.serial and self.serial.is_open:
#             self.serial.close()
#             print("✅ 夹爪串口已关闭")


class RealSenseCamera:
    """RealSense相机接口"""
    
    def __init__(self, config: CameraConfig):
        self.config = config
        self.pipeline: Optional[rs.pipeline] = None
    
    def initialize(self) -> bool:
        """初始化相机"""
        try:
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(
                rs.stream.color,
                self.config.width,
                self.config.height,
                getattr(rs.format, self.config.format),
                self.config.fps
            )
            self.pipeline.start(config)
            print("✅ RealSense相机初始化成功")
            return True
        except Exception as e:
            print(f"❌ 相机初始化失败: {e}")
            return False
    
    def get_frame(self) -> Optional[np.ndarray]:
        """获取彩色图像帧"""
        if not self.pipeline:
            return None
        
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            color_frame = frames.get_color_frame()
            if not color_frame:
                return None
            return np.asanyarray(color_frame.get_data())
        except Exception as e:
            print(f"⚠️ 获取图像帧失败: {e}")
            return None
    
    def stop(self) -> None:
        """停止相机"""
        if self.pipeline:
            self.pipeline.stop()
            print("✅ 相机已停止")


# ============================================================================
# 视觉检测模块
# ============================================================================

@dataclass
class DetectionResult:
    """检测结果"""
    corners: np.ndarray  # 4x2
    centroid: Tuple[int, int]
    debug_image: np.ndarray


class RedSquareDetector:
    """红色正方形检测器"""
    
    def __init__(self, config: VisionConfig):
        self.config = config
        self._morph_kernel = np.ones(
            (config.morph_kernel_size, config.morph_kernel_size),
            np.uint8
        )
    
    def detect(self, image: np.ndarray) -> Optional[DetectionResult]:
        """
        检测红色正方形
        
        Args:
            image: BGR格式图像
            
        Returns:
            检测结果或None
        """
        debug_img = image.copy()
        
        # HSV颜色空间转换
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 红色双阈值掩码
        mask1 = cv2.inRange(hsv, self.config.red_lower_1, self.config.red_upper_1)
        mask2 = cv2.inRange(hsv, self.config.red_lower_2, self.config.red_upper_2)
        mask = cv2.bitwise_or(mask1, mask2)
        
        # 形态学降噪
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._morph_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._morph_kernel)
        
        # 轮廓检测
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        
        # 选择最大轮廓
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        if area < self.config.min_contour_area:
            return None
        
        # 多边形逼近
        perimeter = cv2.arcLength(largest_contour, True)
        approx = cv2.approxPolyDP(
            largest_contour,
            self.config.approx_epsilon * perimeter,
            True
        )
        
        # 获取角点
        if len(approx) == 4:
            corners = approx.reshape(4, 2)
        else:
            # 回退到外接矩形
            x, y, w, h = cv2.boundingRect(largest_contour)
            corners = np.array([
                [x, y], [x + w, y],
                [x + w, y + h], [x, y + h]
            ])
        
        # 角点排序(左上、右上、右下、左下)
        corners = self._order_corners(corners)
        
        # 计算质心
        moments = cv2.moments(largest_contour)
        if moments["m00"] != 0:
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
        else:
            cx, cy = corners.mean(axis=0).astype(int)
        centroid = (cx, cy)
        
        # 绘制调试信息
        self._draw_debug_info(debug_img, corners, centroid)
        
        return DetectionResult(corners, centroid, debug_img)
    
    @staticmethod
    def _order_corners(corners: np.ndarray) -> np.ndarray:
        """按左上、右上、右下、左下顺序排列角点"""
        sorted_by_y = corners[np.argsort(corners[:, 1])]
        top_two = sorted_by_y[:2]
        bottom_two = sorted_by_y[2:]
        
        top_sorted = top_two[np.argsort(top_two[:, 0])]
        bottom_sorted = bottom_two[np.argsort(bottom_two[:, 0])]
        
        return np.vstack([top_sorted, bottom_sorted[::-1]])
    
    @staticmethod
    def _draw_debug_info(image: np.ndarray, corners: np.ndarray,
                        centroid: Tuple[int, int]) -> None:
        """在图像上绘制调试信息"""
        # 绘制角点
        for i, (x, y) in enumerate(corners):
            cv2.circle(image, (int(x), int(y)), 5, (0, 255, 0), -1)
            cv2.putText(image, str(i), (int(x) + 5, int(y) - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # 绘制质心
        cv2.circle(image, centroid, 6, (255, 0, 0), -1)
        cv2.putText(image, f"C({centroid[0]},{centroid[1]})",
                   (centroid[0] + 8, centroid[1] + 8),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)


class FeatureTracker:
    """特征追踪器"""
    
    def __init__(self, detector: RedSquareDetector, vision_config: VisionConfig):
        self.detector = detector
        self.config = vision_config
        
        self.reference_centroid: Optional[Tuple[int, int]] = None
        self.reference_corners: Optional[np.ndarray] = None
        self.is_initialized = False
        self._frame_counter = 0
    
    def capture_reference(self, image: np.ndarray) -> bool:
        """捕获参考帧"""
        result = self.detector.detect(image)
        if result is None:
            print("❌ 未检测到红色方块")
            return False
        
        self.reference_centroid = result.centroid
        self.reference_corners = result.corners
        self.is_initialized = True
        
        # 保存参考图像
        cv2.imwrite("reference.jpg", result.debug_image)
        print(f"✅ 参考帧已捕获: 质心={result.centroid}")
        
        return True
    
    def compute_offset(self, image: np.ndarray) -> Optional[Tuple[int, int, np.ndarray]]:
        """
        计算像素偏移
        
        Returns:
            (dx, dy, debug_image) 或 None
        """
        if not self.is_initialized:
            return None
        
        result = self.detector.detect(image)
        if result is None:
            return None
        
        # 计算偏移
        dx = result.centroid[0] - self.reference_centroid[0]
        dy = result.centroid[1] - self.reference_centroid[1]
        
        # 绘制参考点和偏移向量
        debug_img = result.debug_image
        ref_pos = self.reference_centroid
        cur_pos = result.centroid
        
        cv2.drawMarker(debug_img, ref_pos, (0, 255, 255),
                      cv2.MARKER_CROSS, 12, 2)
        cv2.arrowedLine(debug_img, ref_pos, cur_pos, (0, 255, 255), 2, tipLength=0.25)
        cv2.putText(debug_img, f"Offset: dx={dx}px dy={dy}px",
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # 定期保存调试图像
        if self.config.debug_save_interval > 0:
            self._frame_counter += 1
            if self._frame_counter % self.config.debug_save_interval == 0:
                cv2.imwrite("tracking_debug.jpg", debug_img)
        
        return dx, dy, debug_img


# ============================================================================
# 运动控制模块
# ============================================================================

class MotionController:
    """机器人运动控制器"""
    
    def __init__(self, robot: JAKARobotController, config: ControlConfig):
        self.robot = robot
        self.config = config
        
        # PID控制器
        self.pid_x = PIDController(
            config.pid_x_kp, config.pid_x_ki, config.pid_x_kd,
            config.max_step_mm, config.pid_integral_limit
        )
        self.pid_y = PIDController(
            config.pid_y_kp, config.pid_y_ki, config.pid_y_kd,
            config.max_step_mm, config.pid_integral_limit
        )
        
        # 状态变量
        self.last_timestamp: Optional[float] = None
        self.ema_dx = 0.0
        self.ema_dy = 0.0
        self.stable_counter = 0
        
        # 异步运动队列(非流式模式)
        self._motion_queue: queue.Queue = queue.Queue(maxsize=1)
        self._motion_stop = threading.Event()
        self._motion_thread: Optional[threading.Thread] = None
    
    def reset(self) -> None:
        """重置控制器状态"""
        self.pid_x.reset()
        self.pid_y.reset()
        self.last_timestamp = time.time()
        self.ema_dx = 0.0
        self.ema_dy = 0.0
        self.stable_counter = 0
    
    def compute_control_command(self, dx: float, dy: float) -> Tuple[float, float]:
        """
        计算控制命令
        
        Args:
            dx, dy: 像素偏移(左负右正, 上负下正)
            
        Returns:
            (delta_x_mm, delta_y_mm): 机器人坐标系下的增量(mm)
        """
        # 指数平滑
        if self.config.use_ema_smooth:
            self.ema_dx = self.config.ema_alpha * dx + (1 - self.config.ema_alpha) * self.ema_dx
            self.ema_dy = self.config.ema_alpha * dy + (1 - self.config.ema_alpha) * self.ema_dy
            smooth_dx, smooth_dy = self.ema_dx, self.ema_dy
        else:
            smooth_dx, smooth_dy = dx, dy
        
        # 死区判断
        if abs(smooth_dx) < self.config.pixel_dead_band and \
           abs(smooth_dy) < self.config.pixel_dead_band:
            # 稳定性计数
            if abs(smooth_dx) < self.config.stable_pixel_band and \
               abs(smooth_dy) < self.config.stable_pixel_band:
                self.stable_counter = min(self.stable_counter + 1, 1000)
            else:
                self.stable_counter = 0
            
            # 稳定时重置PID积分
            if self.config.use_pid and \
               self.stable_counter >= self.config.stable_required_frames:
                self.pid_x.reset()
                self.pid_y.reset()
            
            return 0.0, 0.0
        
        # 计算时间步长
        now = time.time()
        if self.last_timestamp is None:
            dt = self.config.servo_interval
        else:
            dt = now - self.last_timestamp
        self.last_timestamp = now
        
        # 计算控制量
        # 映射规则: 左移(dx<0) → y负; 下移(dy>0) → x正
        if self.config.use_pid:
            cmd_x = self.pid_x.update(smooth_dy, dt)
            cmd_y = self.pid_y.update(smooth_dx, dt)
        else:
            cmd_x = self.config.gain_x_per_pixel * smooth_dy
            cmd_y = self.config.gain_y_per_pixel * smooth_dx
        
        # 限幅
        cmd_x = np.clip(cmd_x, -self.config.max_step_mm, self.config.max_step_mm)
        cmd_y = np.clip(cmd_y, -self.config.max_step_mm, self.config.max_step_mm)
        
        # 最小阈值
        if abs(cmd_x) < self.config.min_cmd_mm:
            cmd_x = 0.0
        if abs(cmd_y) < self.config.min_cmd_mm:
            cmd_y = 0.0
        
        return cmd_x, cmd_y
    
    def execute_move(self, delta_x: float, delta_y: float) -> bool:
        """
        执行运动命令
        
        Args:
            delta_x, delta_y: 机器人坐标系增量(mm)
        """
        if delta_x == 0.0 and delta_y == 0.0:
            return True
        
        if self.config.use_servo_streaming:
            # 流式伺服模式
            try:
                increment = [delta_x, delta_y, 0.0, 0.0, 0.0, 0.0]
                self.robot.motion.servo_p(increment, RobotConstants.INCR)
                return True
            except Exception as e:
                print(f"❌ servo_p执行失败: {e}")
                return False
        else:
            # 异步队列模式
            self._submit_to_queue(delta_x, delta_y)
            return True
    
    def _submit_to_queue(self, delta_x: float, delta_y: float) -> None:
        """提交运动到异步队列"""
        try:
            if self._motion_queue.full():
                self._motion_queue.get_nowait()
            self._motion_queue.put_nowait((delta_x, delta_y))
        except Exception:
            pass
    
    def start_async_motion_worker(self) -> None:
        """启动异步运动线程"""
        if not self.config.use_servo_streaming:
            if self._motion_thread is None or not self._motion_thread.is_alive():
                self._motion_stop.clear()
                self._motion_thread = threading.Thread(
                    target=self._motion_worker,
                    name="motion-worker",
                    daemon=True
                )
                self._motion_thread.start()
    
    def _motion_worker(self) -> None:
        """运动线程工作函数"""
        while not self._motion_stop.is_set():
            try:
                cmd = self._motion_queue.get(timeout=0.05)
            except queue.Empty:
                continue
            
            # 获取最新命令
            try:
                while True:
                    cmd = self._motion_queue.get_nowait()
            except queue.Empty:
                pass
            
            delta_x, delta_y = cmd
            self._execute_plane_move(delta_x, delta_y)
    
    def _execute_plane_move(self, delta_x: float, delta_y: float) -> bool:
        """执行平面运动"""
        try:
            # 获取当前TCP位姿
            ret = self.robot.status.get_tcp_position()
            if ret[0] != 0:
                return False
            
            # 计算目标位姿
            current_pose = ret[1]
            target_pose = current_pose.copy()
            target_pose[0] += delta_x
            target_pose[1] += delta_y
            # 保持Z高度
            # target_pose[2] = self.config.plane_z (如需要)
            
            # 逆解
            ik_result = self.robot.coordinate.kine_inverse(
                self.robot.status.get_joint_position()[1],
                target_pose
            )
            if ik_result[0] != 0:
                return False
            
            # 执行运动
            move_result = self.robot.motion.joint_move(
                ik_result[1],
                RobotConstants.ABS,
                True,
                0.6
            )
            return move_result[0] == 0
            
        except Exception as e:
            print(f"❌ 平面运动执行失败: {e}")
            return False
    
    def stop_async_worker(self) -> None:
        """停止异步运动线程"""
        if self._motion_thread is not None:
            self._motion_stop.set()
            self._motion_thread.join(timeout=1.0)


# ============================================================================
# 主系统
# ============================================================================

class VisualServoSystem:
    """视觉伺服系统主控"""
    
    def __init__(self,
                 robot_config: RobotConfig,
                #  gripper_config: GripperConfig,
                 camera_config: CameraConfig,
                 control_config: ControlConfig,
                 vision_config: VisionConfig):
        
        # 配置
        self.robot_cfg = robot_config
        self.control_cfg = control_config
        
        # 硬件
        self.robot: Optional[JAKARobotController] = None
        # self.gripper = Gripper(gripper_config)
        self.camera = RealSenseCamera(camera_config)
        
        # 视觉与控制
        self.detector = RedSquareDetector(vision_config)
        self.tracker = FeatureTracker(self.detector, vision_config)
        self.motion_controller: Optional[MotionController] = None
        
        # 状态
        self.state = SystemState.IDLE
        self.last_offset = (0, 0)
        self.last_command = (0.0, 0.0)
    
    def initialize(self) -> bool:
        """初始化所有组件"""
        print("\n" + "="*50)
        print("视觉伺服系统初始化")
        print("="*50 + "\n")
        
        # 机器人
        print("🤖 初始化机器人...")
        self.robot = JAKARobotController(self.robot_cfg.ip)
        if not self.robot.utils.initialize_robot():
            print("❌ 机器人初始化失败")
            return False
        
        # 回到初始位置
        ret = self.robot.motion.joint_move(
            self.robot_cfg.home_joint_rad,
            RobotConstants.ABS,
            True,
            0.2
        )
        if ret[0] != 0:
            print(f"❌ 回到初始位置失败: {ret[0]}")
            return False
        print("✅ 机器人就绪")
        
        # 夹爪
        # print("\n🤏 初始化夹爪...")
        # if not self.gripper.initialize():
        #     return False
        # self.gripper.open()
        
        # 相机
        print("\n📷 初始化相机...")
        if not self.camera.initialize():
            return False
        
        # 运动控制器
        self.motion_controller = MotionController(self.robot, self.control_cfg)
        self.motion_controller.start_async_motion_worker()
        
        # 配置伺服滤波器
        if self.control_cfg.use_servo_streaming and self.control_cfg.use_servo_filter:
            self._configure_servo_filter()
        
        print("\n" + "="*50)
        print("✅ 系统初始化完成")
        print("="*50 + "\n")
        return True
    
    def _configure_servo_filter(self) -> None:
        """配置伺服滤波器"""
        filter_type = self.control_cfg.servo_filter_type
        
        try:
            if filter_type == 'NONE':
                ret = self.robot.motion.servo_move_use_none_filter()
            elif filter_type == 'CARTESIAN_NLF':
                ret = self.robot.motion.servo_move_use_carte_NLF(
                    self.control_cfg.cart_max_vp,
                    self.control_cfg.cart_max_ap,
                    self.control_cfg.cart_max_jp,
                    self.control_cfg.cart_max_vr,
                    self.control_cfg.cart_max_ar,
                    self.control_cfg.cart_max_jr
                )
            else:
                print(f"⚠️ 未知滤波器类型: {filter_type}")
                return
            
            if ret[0] == 0:
                print(f"✅ 伺服滤波器已配置: {filter_type}")
            else:
                print(f"⚠️ 滤波器配置失败: {ret}")
                
        except Exception as e:
            print(f"⚠️ 滤波器配置异常: {e}")
    
    def run(self) -> None:
        """主运行循环"""
        if not self.initialize():
            print("❌ 系统初始化失败")
            return
        
        self._print_usage()
        
        try:
            cv2.namedWindow('Visual Servo', cv2.WINDOW_NORMAL)
            
            while True:
                # 获取图像帧
                frame = self.camera.get_frame()
                if frame is None:
                    time.sleep(0.05)
                    continue
                
                # 处理当前帧
                display_frame = self._process_frame(frame)
                
                # 显示
                cv2.imshow('Visual Servo', display_frame)
                cv2.waitKey(1)
                
                # 键盘输入
                if msvcrt.kbhit():
                    key = msvcrt.getch().decode('utf-8', errors='ignore')
                    if not self._handle_key(key):
                        break
                
                # 运行时延时
                if self.state != SystemState.TRACKING:
                    time.sleep(0.05)
                else:
                    time.sleep(self.control_cfg.servo_interval)
                    
        except KeyboardInterrupt:
            print("\n⚠️ 检测到中断信号")
        except Exception as e:
            print(f"❌ 运行时错误: {e}")
            traceback.print_exc()
        finally:
            self.cleanup()
    
    def _process_frame(self, frame: np.ndarray) -> np.ndarray:
        """
        处理单帧图像
        
        Returns:
            用于显示的图像
        """
        # 检测
        result = self.detector.detect(frame)
        display = result.debug_image if result else frame.copy()
        
        # 如果已初始化追踪
        if self.tracker.is_initialized:
            offset_result = self.tracker.compute_offset(frame)
            if offset_result:
                dx, dy, display = offset_result
                self.last_offset = (dx, dy)
                
                # 如果处于追踪状态，执行伺服
                if self.state == SystemState.TRACKING:
                    self._execute_servo_step(dx, dy)
        
        # 叠加UI信息
        self._draw_ui_overlay(display)
        
        return display
    
    def _execute_servo_step(self, dx: int, dy: int) -> None:
        """执行单步伺服"""
        if self.motion_controller is None:
            return
        
        # 计算控制命令
        cmd_x, cmd_y = self.motion_controller.compute_control_command(dx, dy)
        self.last_command = (cmd_x, cmd_y)
        
        # 执行运动
        if cmd_x != 0.0 or cmd_y != 0.0:
            self.motion_controller.execute_move(cmd_x, cmd_y)
    
    def _draw_ui_overlay(self, image: np.ndarray) -> None:
        """在图像上绘制UI信息"""
        h, w = image.shape[:2]
        y = 60
        scale = 0.6
        thickness = 2
        color_status = (0, 255, 0) if self.state == SystemState.TRACKING else (0, 165, 255)
        
        # 状态
        cv2.putText(image, f"State: {self.state.value.upper()}", 
                   (10, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color_status, thickness)
        y += 25
        
        # 偏移
        if self.tracker.is_initialized:
            dx, dy = self.last_offset
            cv2.putText(image, f"Offset: dx={dx:+4d}px  dy={dy:+4d}px", 
                       (10, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 0), thickness)
            y += 25
            
            # 命令
            cmd_x, cmd_y = self.last_command
            cv2.putText(image, f"Command: X={cmd_x:+6.2f}mm  Y={cmd_y:+6.2f}mm", 
                       (10, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 0), thickness)
            y += 25
        
        # 控制参数
        if self.control_cfg.use_pid:
            cv2.putText(image, 
                       f"PID: Kp={self.control_cfg.pid_x_kp:.3f} Ki={self.control_cfg.pid_x_ki:.3f} Kd={self.control_cfg.pid_x_kd:.3f}", 
                       (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 255), 1)
        else:
            cv2.putText(image, 
                       f"Gain: X={self.control_cfg.gain_x_per_pixel:.3f} Y={self.control_cfg.gain_y_per_pixel:.3f}", 
                       (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        y += 22
        
        # 模式
        mode = "Servo Stream" if self.control_cfg.use_servo_streaming else "Async Queue"
        cv2.putText(image, f"Mode: {mode}", 
                   (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 255, 180), 1)
    
    def _handle_key(self, key: str) -> bool:
        """
        处理键盘输入
        
        Returns:
            True继续运行，False退出
        """
        if key == 'c':
            self._handle_capture()
        elif key == 's':
            self._handle_start_stop()
        elif key == 'p':
            self._handle_pause()
        elif key == 'i':
            self._handle_return_home()
        elif key == 'q':
            print("👋 退出程序...")
            return False
        
        return True
    
    def _handle_capture(self) -> None:
        """处理捕获命令"""
        print("\n📸 捕获参考帧...")
        self.state = SystemState.CAPTURING
        
        frame = self.camera.get_frame()
        if frame is not None and self.tracker.capture_reference(frame):
            self.state = SystemState.IDLE
        else:
            print("❌ 捕获失败")
            self.state = SystemState.ERROR
    
    def _handle_start_stop(self) -> None:
        """处理启动/停止命令"""
        if not self.tracker.is_initialized:
            print("⚠️ 请先按 'c' 捕获参考帧")
            return
        
        if self.state == SystemState.TRACKING:
            # 停止追踪
            self.state = SystemState.IDLE
            print("⏸️ 追踪已停止")
            
            if self.control_cfg.use_servo_streaming:
                try:
                    self.robot.motion.servo_move_enable(False)
                    print("✅ 伺服模式已关闭")
                except Exception as e:
                    print(f"⚠️ 关闭伺服失败: {e}")
        else:
            # 启动追踪
            self.state = SystemState.TRACKING
            print("▶️ 追踪已启动")
            
            # 重置控制器
            if self.motion_controller:
                self.motion_controller.reset()
            
            # 启用伺服模式
            if self.control_cfg.use_servo_streaming:
                try:
                    ret = self.robot.motion.servo_move_enable(True)
                    if ret[0] == 0:
                        print("✅ 伺服模式已启用")
                    else:
                        print(f"⚠️ 伺服启用失败: {ret}")
                except Exception as e:
                    print(f"⚠️ 伺服启用异常: {e}")
    
    def _handle_pause(self) -> None:
        """处理暂停命令"""
        if self.state == SystemState.TRACKING:
            self.state = SystemState.PAUSED
            print("⏸️ 系统已暂停")
            
            if self.control_cfg.use_servo_streaming:
                try:
                    self.robot.motion.servo_move_enable(False)
                except Exception:
                    pass
        else:
            print("ℹ️ 系统未在运行")
    
    def _handle_return_home(self) -> None:
        """处理回初始位置命令"""
        print("\n🏠 返回初始位置...")
        
        # 先停止追踪
        if self.state == SystemState.TRACKING:
            self._handle_start_stop()
        
        ret = self.robot.motion.joint_move(
            self.robot_cfg.home_joint_rad,
            RobotConstants.ABS,
            True,
            0.2
        )
        
        if ret[0] == 0:
            print("✅ 已返回初始位置")
        else:
            print(f"❌ 返回失败: {ret[0]}")
    
    def cleanup(self) -> None:
        """清理资源"""
        print("\n🧹 清理资源...")
        
        # 停止伺服
        if self.robot and self.control_cfg.use_servo_streaming:
            try:
                self.robot.motion.servo_move_enable(False)
                self.robot.motion.motion_abort()
            except Exception:
                pass
        
        # 停止运动线程
        if self.motion_controller:
            self.motion_controller.stop_async_worker()
        
        # 关闭硬件
        # self.gripper.cleanup()
        self.camera.stop()
        
        if self.robot:
            self.robot.utils.safe_shutdown()
        
        # 关闭窗口
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        
        print("✅ 清理完成")
    
    @staticmethod
    def _print_usage() -> None:
        """打印使用说明"""
        print("\n" + "="*50)
        print("操作指南")
        print("="*50)
        print("  [c] 捕获参考帧")
        print("  [s] 启动/停止追踪")
        print("  [p] 暂停")
        print("  [i] 返回初始位置")
        print("  [q] 退出程序")
        print("="*50 + "\n")


# ============================================================================
# 程序入口
# ============================================================================

def main():
    """导入机械臂依赖库"""
    __common.init_env()
    import jkrc
    """主函数"""
    # 创建配置
    robot_config = RobotConfig()
    # gripper_config = GripperConfig()
    camera_config = CameraConfig()
    control_config = ControlConfig()
    vision_config = VisionConfig()
    
    # 创建系统
    system = VisualServoSystem(
        robot_config,
        # gripper_config,
        camera_config,
        control_config,
        vision_config
    )
    
    # 运行
    system.run()


if __name__ == '__main__':
    main()