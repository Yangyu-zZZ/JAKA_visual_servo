"""
JAKA机械臂IO控制模块
包含数字输入输出、模拟输入输出、扩展IO等功能
"""

from typing import Tuple


class IOControl:
    """机器人IO控制功能"""
    
    def __init__(self, robot):
        """
        初始化IO控制模块
        
        Args:
            robot: JAKA机器人对象(jkrc.RC实例)
        """
        self.robot = robot
    
    # ==================== 数字IO ====================
    
    def set_digital_output(self, iotype: int, index: int, 
                          value: int) -> Tuple[int, ...]:
        """
        设置数字输出(DO)
        
        Args:
            iotype: IO类型 IO_CABINET/IO_TOOL/IO_EXTEND
            index: DO索引
            value: 输出值(0/1)
            
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.set_digital_output(iotype, index, value)
    
    def get_digital_output(self, iotype: int, index: int) -> Tuple[int, int]:
        """
        获取数字输出(DO)状态
        
        Args:
            iotype: IO类型
            index: DO索引
            
        Returns:
            (0, value): 成功, value为0或1
            其他: 失败
        """
        return self.robot.get_digital_output(iotype, index)
    
    def get_digital_input(self, iotype: int, index: int) -> Tuple[int, int]:
        """
        获取数字输入(DI)状态
        
        Args:
            iotype: IO类型
            index: DI索引
            
        Returns:
            (0, value): 成功, value为0或1
            其他: 失败
        """
        return self.robot.get_digital_input(iotype, index)
    
    # ==================== 模拟IO ====================
    
    def set_analog_output(self, iotype: int, index: int, 
                         value: float) -> Tuple[int, ...]:
        """
        设置模拟输出(AO)
        
        Args:
            iotype: IO类型
            index: AO索引
            value: 输出值(浮点数)
            
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.set_analog_output(iotype, index, value)
    
    def get_analog_output(self, iotype: int, index: int) -> Tuple[int, float]:
        """
        获取模拟输出(AO)值
        
        Args:
            iotype: IO类型
            index: AO索引
            
        Returns:
            (0, value): 成功, value为浮点数
            其他: 失败
        """
        return self.robot.get_analog_output(iotype, index)
    
    def get_analog_input(self, iotype: int, index: int) -> Tuple[int, float]:
        """
        获取模拟输入(AI)值
        
        Args:
            iotype: IO类型
            index: AI索引
            
        Returns:
            (0, value): 成功, value为浮点数
            其他: 失败
        """
        return self.robot.get_analog_input(iotype, index)
    
    # ==================== 扩展IO ====================
    
    def is_extio_running(self) -> Tuple[int, int]:
        """
        查询扩展IO是否运行
        
        Returns:
            (0, status): 成功, status=1运行, 0未运行
            其他: 失败
        """
        return self.robot.is_extio_running()
