"""
JAKA机械臂安全管理模块
包含碰撞检测、限位检查、错误处理等安全功能
"""

from typing import Tuple, Any


class SafetyManager:
    """机器人安全管理功能"""
    
    def __init__(self, robot):
        """
        初始化安全管理模块
        
        Args:
            robot: JAKA机器人对象(jkrc.RC实例)
        """
        self.robot = robot
    
    # ==================== 限位检查 ====================
    
    def is_on_limit(self) -> Tuple[int, int]:
        """
        查询是否超出限位
        
        Returns:
            (0, state): 成功, state=1超限, 0正常
            其他: 失败
        """
        return self.robot.is_on_limit()
    
    # ==================== 碰撞检测 ====================
    
    def is_in_collision(self) -> Tuple[int, int]:
        """
        查询是否处于碰撞保护模式
        
        Returns:
            (0, state): 成功, state=1碰撞, 0正常
            其他: 失败
        """
        return self.robot.is_in_collision()
    
    def collision_recover(self) -> Tuple[int, ...]:
        """
        从碰撞保护模式恢复
        
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.collision_recover()
    
    def set_collision_level(self, level: int) -> Tuple[int, ...]:
        """
        设置碰撞等级
        
        Args:
            level: 碰撞等级
                0: 关闭碰撞检测
                1: 25N阈值
                2: 50N阈值
                3: 75N阈值
                4: 100N阈值
                5: 125N阈值
                
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.set_collision_level(level)
    
    def get_collision_level(self) -> Tuple[int, int]:
        """
        获取碰撞等级
        
        Returns:
            (0, level): 成功, level为0-5
            其他: 失败
        """
        return self.robot.get_collision_level()
    
    # ==================== 错误处理 ====================
    
    def get_last_error(self) -> Tuple[int, Any]:
        """
        获取最新错误码
        
        Returns:
            (0, error): 成功, error为错误信息
            其他: 失败
            
        注意: 需先调用set_errorcode_file_path设置错误码文件路径
        """
        return self.robot.get_last_error()
    
    def set_errorcode_file_path(self, path: str) -> Tuple[int, ...]:
        """
        设置错误码文件路径
        
        Args:
            path: 错误码文件路径(不能包含中文)
            
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.set_errorcode_file_path(path)
    
    def clear_error(self) -> Tuple[int, ...]:
        """
        清除错误状态
        
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.clear_error()
    
    # ==================== 网络异常处理 ====================
    
    def set_network_exception_handle(self, millisecond: int, 
                                    mnt: int) -> Tuple[int, ...]:
        """
        设置网络异常时机器人动作
        
        Args:
            millisecond: 时间参数,单位ms
            mnt: 动作类型
                0: 保持原运动
                1: 暂停运动
                2: 终止运动
                
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.set_network_exception_handle(millisecond, mnt)
    
    # ==================== 安装角度 ====================
    
    def set_installation_angle(self, anglex: float, angley: float) -> Tuple[int, ...]:
        """
        设置机器人安装角度
        
        Args:
            anglex: x方向安装角度,范围[0,180]度
            angley: z方向安装角度,范围[0,360]度
            
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.set_installation_angle(anglex, angley)
    
    def get_installation_angle(self) -> Tuple[int, list]:
        """
        获取机器人安装角度
        
        Returns:
            (0, [qs,qx,qy,qz,rx,ry,rz]): 成功
            四元数[qs,qx,qy,qz]和欧拉角[rx,ry,rz]
        """
        return self.robot.get_installation_angle()
