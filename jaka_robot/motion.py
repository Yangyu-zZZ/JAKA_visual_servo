"""
JAKA机械臂运动控制模块
包含关节运动、直线运动、圆弧运动、伺服运动等功能
"""

from typing import List, Tuple


class MotionControl:
    """机器人运动控制功能"""
    
    def __init__(self, robot):
        """
        初始化运动控制模块
        
        Args:
            robot: JAKA机器人对象(jkrc.RC实例)
        """
        self.robot = robot
    
    # ==================== 手动模式运动 ====================
    
    def jog(self, aj_num: int, move_mode: int, coord_type: int, 
            jog_vel: float, pos_cmd: float = 0) -> Tuple[int, ...]:
        """
        手动模式下机器人运动(非阻塞)
        
        Args:
            aj_num: 轴号(0-5)或笛卡尔方向(0-5对应x,y,z,rx,ry,rz)
            move_mode: 运动模式 0=绝对运动, 1=增量运动, 2=连续运动
            coord_type: 坐标系类型 COORD_BASE/COORD_JOINT/COORD_TOOL
            jog_vel: 速度 关节单位rad/s, 笛卡尔单位mm/s
            pos_cmd: 位置指令 关节单位rad, 笛卡尔单位mm(绝对运动时可忽略)
            
        Returns:
            (0,): 成功
            其他: 失败
            
        注意: 需要连续发送,发送周期建议与控制周期一致(8ms)
        """
        return self.robot.jog(aj_num, move_mode, coord_type, jog_vel, pos_cmd)
    
    def jog_stop(self, joint_num: int = -1) -> Tuple[int, ...]:
        """
        停止手动模式运动
        
        Args:
            joint_num: 要停止的关节轴号(0-5), -1表示停止所有轴
            
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.jog_stop(joint_num)
    
    # ==================== 关节空间运动 ====================
    
    def joint_move(self, joint_pos: List[float], move_mode: int, 
                   is_block: bool, speed: float) -> Tuple[int, ...]:
        """
        关节空间运动
        
        Args:
            joint_pos: 目标关节角度列表[j1,j2,j3,j4,j5,j6],单位rad
            move_mode: 0=绝对运动, 1=增量运动
            is_block: True=阻塞(运动完成才返回), False=非阻塞
            speed: 运动速度,单位rad/s
            
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.joint_move(joint_pos, move_mode, is_block, speed)
    
    def joint_move_extend(self, joint_pos: List[float], move_mode: int, 
                         is_block: bool, speed: float, acc: float, 
                         tol: float) -> Tuple[int, ...]:
        """
        扩展关节空间运动(带加速度和终点误差)
        
        Args:
            joint_pos: 目标关节角度列表,单位rad
            move_mode: 0=绝对运动, 1=增量运动
            is_block: True=阻塞, False=非阻塞
            speed: 运动速度,单位rad/s
            acc: 角加速度,单位rad/s²
            tol: 终点误差
            
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.joint_move_extend(joint_pos, move_mode, is_block, 
                                           speed, acc, tol)
    
    # ==================== 笛卡尔空间运动 ====================
    
    def linear_move(self, end_pos: List[float], move_mode: int, 
                    is_block: bool, speed: float) -> Tuple[int, ...]:
        """
        末端直线运动
        
        Args:
            end_pos: 目标位姿[x,y,z,rx,ry,rz],位置单位mm,姿态单位rad
            move_mode: 0=绝对运动, 1=增量运动
            is_block: True=阻塞, False=非阻塞
            speed: 运动速度,单位mm/s
            
        Returns:
            (0,): 成功
            (-4,): 逆解失败
            其他: 其他错误
        """
        return self.robot.linear_move(end_pos, move_mode, is_block, speed)
    
    def linear_move_extend(self, end_pos: List[float], move_mode: int, 
                          is_block: bool, speed: float, acc: float, 
                          tol: float) -> Tuple[int, ...]:
        """
        扩展末端直线运动(带加速度和终点误差)
        
        Args:
            end_pos: 目标位姿[x,y,z,rx,ry,rz]
            move_mode: 0=绝对运动, 1=增量运动
            is_block: True=阻塞, False=非阻塞
            speed: 运动速度,单位mm/s
            acc: 加速度,单位mm/s²
            tol: 终点误差
            
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.linear_move_extend(end_pos, move_mode, is_block, 
                                            speed, acc, tol)
    
    # ==================== 圆弧运动 ====================
    
    def circular_move(self, end_pos: List[float], mid_pos: List[float], 
                     move_mode: int, is_block: bool, speed: float, 
                     acc: float, tol: float) -> Tuple[int, ...]:
        """
        末端圆弧运动
        
        Args:
            end_pos: 终点位姿[x,y,z,rx,ry,rz]
            mid_pos: 中间点位姿[x,y,z,rx,ry,rz]
            move_mode: 0=绝对运动, 1=增量运动
            is_block: True=阻塞, False=非阻塞
            speed: 运动速度,单位mm/s
            acc: 加速度,单位mm/s²
            tol: 终点误差
            
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.circular_move(end_pos, mid_pos, move_mode, 
                                       is_block, speed, acc, tol)
    
    def circular_move_extend(self, end_pos: List[float], mid_pos: List[float],
                            move_mode: int, is_block: bool, speed: float,
                            acc: float, tol: float, circle_cnt: int) -> Tuple[int, ...]:
        """
        扩展末端圆弧运动(可指定圈数)
        
        Args:
            end_pos: 终点位姿
            mid_pos: 中间点位姿
            move_mode: 0=绝对运动, 1=增量运动
            is_block: True=阻塞, False=非阻塞
            speed: 运动速度,单位mm/s
            acc: 加速度,单位mm/s²
            tol: 终点误差
            circle_cnt: 圆弧运动圈数
            
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.circular_move_extend(end_pos, mid_pos, move_mode,
                                              is_block, speed, acc, tol, 
                                              circle_cnt)
    
    # ==================== 运动控制 ====================
    
    def motion_abort(self) -> Tuple[int, ...]:
        """
        立即终止当前运动
        
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.motion_abort()
    
    def is_in_pos(self) -> Tuple[int, int]:
        """
        查询机器人运动是否停止
        
        Returns:
            (0, state): 成功, state=1表示停止, 0表示运动中
            其他: 失败
        """
        return self.robot.is_in_pos()
    
    def set_block_wait_timeout(self, seconds: float) -> Tuple[int, ...]:
        """
        设置阻塞运动超时时间
        
        Args:
            seconds: 超时时间(秒),必须>0.5
            
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.set_block_wait_timeout(seconds)
    
    # ==================== 伺服运动(高级) ====================
    
    def servo_move_enable(self, enable: bool) -> Tuple[int, ...]:
        """
        伺服模式使能
        
        Args:
            enable: True=进入伺服模式, False=退出
            
        Returns:
            (0,): 成功
            其他: 失败
            
        注意: 使用servo_j/servo_p前必须先使能
        """
        return self.robot.servo_move_enable(enable)
    
    def servo_j(self, joint_pos: List[float], move_mode: int) -> Tuple[int, ...]:
        """
        关节空间伺服运动
        
        Args:
            joint_pos: 目标关节角度,单位rad
            move_mode: 0=绝对运动, 1=增量运动
            
        Returns:
            (0,): 成功
            其他: 失败
            
        注意:
        1. 需先调用servo_move_enable(True)
        2. 建议发送周期8ms,需连续发送
        3. 关节速度不能超过180°/s
        4. 需要用户自行规划轨迹
        """
        return self.robot.servo_j(joint_pos, move_mode)
    
    def servo_j_extend(self, joint_pos: List[float], move_mode: int, 
                      step_num: int) -> Tuple[int, ...]:
        """
        扩展关节空间伺服运动(可设置倍分周期)
        
        Args:
            joint_pos: 目标关节角度
            move_mode: 0=绝对运动, 1=增量运动
            step_num: 倍分周期,运动周期=step_num*8ms,step_num>=1
            
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.servo_j_extend(joint_pos, move_mode, step_num)
    
    def servo_p(self, cartesian_pose: List[float], move_mode: int) -> Tuple[int, ...]:
        """
        笛卡尔空间伺服运动
        
        Args:
            cartesian_pose: 目标位姿[x,y,z,rx,ry,rz]
            move_mode: 0=绝对运动, 1=增量运动
            
        Returns:
            (0,): 成功
            其他: 失败
            
        注意:
        1. 需先调用servo_move_enable(True)
        2. 建议发送周期8ms,需连续发送
        3. 需要用户自行规划轨迹
        """
        return self.robot.servo_p(cartesian_pose, move_mode)
    
    def servo_p_extend(self, cartesian_pose: List[float], move_mode: int, 
                      step_num: int) -> Tuple[int, ...]:
        """
        扩展笛卡尔空间伺服运动
        
        Args:
            cartesian_pose: 目标位姿
            move_mode: 0=绝对运动, 1=增量运动
            step_num: 倍分周期,运动周期=step_num*8ms
            
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.servo_p_extend(cartesian_pose, move_mode, step_num)
    
    # ==================== 伺服滤波器设置 ====================
    
    def servo_move_use_none_filter(self) -> Tuple[int, ...]:
        """
        伺服模式禁用滤波器
        
        Returns:
            (0,): 成功
            其他: 失败
            
        注意: 需在退出伺服模式后设置
        """
        return self.robot.servo_move_use_none_filter()
    
    def servo_move_use_joint_LPF(self, cutoff_freq: float) -> Tuple[int, ...]:
        """
        伺服模式关节空间一阶低通滤波
        
        Args:
            cutoff_freq: 截止频率
            
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.servo_move_use_joint_LPF(cutoff_freq)
    
    def servo_move_use_joint_NLF(self, max_vr: float, max_ar: float, 
                                 max_jr: float) -> Tuple[int, ...]:
        """
        伺服模式关节空间非线性滤波
        
        Args:
            max_vr: 速度上限,单位°/s
            max_ar: 加速度上限,单位°/s²
            max_jr: 加加速度上限,单位°/s³
            
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.servo_move_use_joint_NLF(max_vr, max_ar, max_jr)
    
    def servo_move_use_carte_NLF(self, max_vp: float, max_ap: float, 
                                 max_jp: float, max_vr: float, 
                                 max_ar: float, max_jr: float) -> Tuple[int, ...]:
        """
        伺服模式笛卡尔空间非线性滤波
        
        Args:
            max_vp: 移动速度上限,单位mm/s
            max_ap: 移动加速度上限,单位mm/s²
            max_jp: 移动加加速度上限,单位mm/s³
            max_vr: 姿态速度上限,单位°/s
            max_ar: 姿态加速度上限,单位°/s²
            max_jr: 姿态加加速度上限,单位°/s³
            
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.servo_move_use_carte_NLF(max_vp, max_ap, max_jp,
                                                   max_vr, max_ar, max_jr)
    
    def servo_move_use_joint_MMF(self, max_buf: int, kp: float, 
                                 kv: float, ka: float) -> Tuple[int, ...]:
        """
        伺服模式关节空间多阶均值滤波
        
        Args:
            max_buf: 缓冲区大小
            kp: 位置滤波系数
            kv: 速度滤波系数
            ka: 加速度滤波系数
            
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.servo_move_use_joint_MMF(max_buf, kp, kv, ka)
    
    def servo_speed_foresight(self, max_buf: int, kp: float) -> Tuple[int, ...]:
        """
        伺服模式速度前瞻参数设置
        
        Args:
            max_buf: 缓冲区大小
            kp: 加速度滤波系数
            
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.servo_speed_foresight(max_buf, kp)
