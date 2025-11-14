"""
JAKA机械臂程序控制模块
包含程序加载、运行、暂停、终止等功能
"""

from typing import Tuple


class ProgramControl:
    """机器人程序控制功能"""
    
    def __init__(self, robot):
        """
        初始化程序控制模块
        
        Args:
            robot: JAKA机器人对象(jkrc.RC实例)
        """
        self.robot = robot
    
    def program_load(self, file_name: str) -> Tuple[int, ...]:
        """
        加载作业程序
        
        Args:
            file_name: 程序名称
            
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.program_load(file_name)
    
    def get_loaded_program(self) -> Tuple[int, str]:
        """
        获取已加载的程序名称
        
        Returns:
            (0, file_name): 成功
            其他: 失败
        """
        return self.robot.get_loaded_program()
    
    def get_current_line(self) -> Tuple[int, int]:
        """
        获取当前程序执行行号
        
        Returns:
            (0, line): 成功
            其他: 失败
        """
        return self.robot.get_current_line()
    
    def program_run(self) -> Tuple[int, ...]:
        """
        运行当前加载的程序
        
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.program_run()
    
    def program_pause(self) -> Tuple[int, ...]:
        """
        暂停当前运行的程序
        
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.program_pause()
    
    def program_resume(self) -> Tuple[int, ...]:
        """
        继续运行暂停的程序
        
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.program_resume()
    
    def program_abort(self) -> Tuple[int, ...]:
        """
        终止当前程序
        
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.program_abort()
    
    def get_program_state(self) -> Tuple[int, int]:
        """
        获取程序执行状态
        
        Returns:
            (0, state): 成功
            state: 0=停止, 1=运行中, 2=暂停
        """
        return self.robot.get_program_state()
    
    def set_rapidrate(self, rapid_rate: float) -> Tuple[int, ...]:
        """
        设置运行速度倍率
        
        Args:
            rapid_rate: 速度倍率(0-1)
            
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.set_rapidrate(rapid_rate)
    
    def get_rapidrate(self) -> Tuple[int, float]:
        """
        获取运行速度倍率
        
        Returns:
            (0, rate): 成功, rate范围0-1
            其他: 失败
        """
        return self.robot.get_rapidrate()
