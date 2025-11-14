"""
JAKA机械臂控制封装包
版本: 2.0.0 (模块化版本)
作者: Based on JAKA Python SDK V2.1.11
说明: 模块化的JAKA机器人控制接口,便于维护和扩展
"""

from .constants import RobotConstants
from .base import BaseControl
from .motion import MotionControl
from .io_control import IOControl
from .coordinate import CoordinateSystem
from .safety import SafetyManager
from .status import StatusQuery
from .program import ProgramControl
from .utils import RobotUtils


class JAKARobotController:
    """JAKA机械臂主控制器类"""
    
    def __init__(self, ip_address: str):
        """
        初始化机器人控制器
        
        Args:
            ip_address: 机器人IP地址,如"10.5.5.100"
        """
        # 导入jkrc模块
        try:
            import jkrc
        except ImportError:
            raise ImportError("无法导入jkrc模块,请确保JAKA SDK已正确安装")
        
        self.ip = ip_address
        self.robot = jkrc.RC(self.ip)
        self.is_connected = False
        
        # 初始化各个功能模块
        self.base = BaseControl(self.robot)
        self.motion = MotionControl(self.robot)
        self.io = IOControl(self.robot)
        self.coordinate = CoordinateSystem(self.robot)
        self.safety = SafetyManager(self.robot)
        self.status = StatusQuery(self.robot)
        self.program = ProgramControl(self.robot)
        self.utils = RobotUtils(self)
    
    # ==================== 便捷方法(直接调用utils) ====================
    
    def initialize_robot(self, wait_time: float = RobotConstants.DEFAULT_POWERUP_TIME) -> bool:
        """
        机器人完整初始化流程(登录→上电→使能)
        
        Args:
            wait_time: 上电等待时间(秒),默认8秒
            
        Returns:
            True: 初始化成功
            False: 初始化失败
        """
        result = self.utils.initialize_robot(wait_time)
        if result:
            self.is_connected = True
        return result
    
    def safe_shutdown(self, wait_time: float = 8.0) -> bool:
        """
        机器人安全关闭流程(下使能→断电→登出)
        
        Args:
            wait_time: 下使能后等待时间(秒)
            
        Returns:
            True: 关闭成功
            False: 关闭失败
        """
        result = self.utils.safe_shutdown(wait_time)
        if result:
            self.is_connected = False
        return result
    
    def move_to_home(self, home_joint: list = None, speed: float = RobotConstants.DEFAULT_JOINT_SPEED) -> bool:
        """移动到HOME位置"""
        return self.utils.move_to_home(home_joint, speed)
    
    def wait_motion_done(self, timeout: float = RobotConstants.DEFAULT_TIMEOUT, 
                        check_interval: float = RobotConstants.DEFAULT_CHECK_INTERVAL) -> bool:
        """等待运动完成"""
        return self.utils.wait_motion_done(timeout, check_interval)
    
    def check_robot_ready(self) -> bool:
        """检查机器人是否就绪"""
        return self.utils.check_robot_ready()
    
    def get_current_pose_info(self) -> dict:
        """获取当前位姿完整信息"""
        return self.utils.get_current_pose_info()
    
    def print_status_summary(self):
        """打印机器人状态摘要"""
        self.utils.print_status_summary()


# 导出主要类和常量
__all__ = [
    'JAKARobotController',
    'RobotConstants',
    'BaseControl',
    'MotionControl',
    'IOControl',
    'CoordinateSystem',
    'SafetyManager',
    'StatusQuery',
    'ProgramControl',
    'RobotUtils'
]

__version__ = '2.0.0'
