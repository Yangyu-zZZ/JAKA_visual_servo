"""
JAKA机械臂基础控制模块
包含登录、上电、使能等基础操作
"""

from typing import Tuple


class BaseControl:
    """机器人基础控制功能"""
    
    def __init__(self, robot):
        """
        初始化基础控制模块
        
        Args:
            robot: JAKA机器人对象(jkrc.RC实例)
        """
        self.robot = robot
    
    def login(self) -> Tuple[int, ...]:
        """
        登录机器人控制器
        
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.login()
    
    def logout(self) -> Tuple[int, ...]:
        """
        注销机器人连接
        
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.logout()
    
    def power_on(self) -> Tuple[int, ...]:
        """
        给机器人上电(约需8秒)
        
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.power_on()
    
    def power_off(self) -> Tuple[int, ...]:
        """
        关闭机器人电源
        
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.power_off()
    
    def enable_robot(self) -> Tuple[int, ...]:
        """
        机器人上使能
        
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.enable_robot()
    
    def disable_robot(self) -> Tuple[int, ...]:
        """
        机器人下使能
        
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.disable_robot()
    
    def shut_down(self) -> Tuple[int, ...]:
        """
        机器人控制柜关机
        
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.shut_down()
    
    def get_sdk_version(self) -> Tuple[int, str]:
        """
        获取SDK版本号
        
        Returns:
            (0, version): 成功,version为版本号字符串
            其他: 失败
        """
        return self.robot.get_sdk_version()
    
    def get_controller_ip(self) -> Tuple[int, list]:
        """
        获取控制器IP地址列表
        
        Returns:
            (0, ip_list): 成功,ip_list为IP地址列表
            其他: 失败
        """
        return self.robot.get_controller_ip()
    
    def drag_mode_enable(self, enable: bool) -> Tuple[int, ...]:
        """
        进入/退出拖拽模式
        
        Args:
            enable: True进入拖拽模式, False退出拖拽模式
            
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.drag_mode_enable(enable)
    
    def is_in_drag_mode(self) -> Tuple[int, int]:
        """
        查询机器人是否处于拖拽模式
        
        Returns:
            (0, state): 成功, state=1表示处于拖拽模式, 0表示不在
            其他: 失败
        """
        return self.robot.is_in_drag_mode()
    
    def set_debug_mode(self, mode: bool) -> Tuple[int, ...]:
        """
        设置SDK调试模式
        
        Args:
            mode: True开启调试模式, False关闭
            
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.set_debug_mode(mode)
    
    def set_SDK_filepath(self, filepath: str) -> Tuple[int, ...]:
        """
        设置SDK日志路径
        
        Args:
            filepath: 日志文件路径
            
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.set_SDK_filepath(filepath)
    
    def set_payload(self, mass: float, centroid: list) -> Tuple[int, ...]:
        """
        设置机器人负载
        
        Args:
            mass: 负载质量,单位kg
            centroid: 质心坐标[x,y,z],单位mm
            
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.set_payload(mass, centroid)
    
    def get_payload(self) -> Tuple[int, Tuple[float, Tuple[float, float, float]]]:
        """
        获取机器人负载
        
        Returns:
            (0, (mass, (x,y,z))): 成功
            mass: 质量(kg)
            (x,y,z): 质心坐标(mm)
        """
        return self.robot.get_payload()
