"""
JAKA机械臂状态查询模块
包含机器人状态、位置信息等查询功能
"""

from typing import List, Tuple


class StatusQuery:
    """机器人状态查询功能"""
    
    def __init__(self, robot):
        """
        初始化状态查询模块
        
        Args:
            robot: JAKA机器人对象(jkrc.RC实例)
        """
        self.robot = robot
    
    def get_robot_status(self) -> Tuple[int, List]:
        """
        获取机器人完整状态数据(24个元素)
        
        Returns:
            (0, status_list): 成功
            status_list包含:
            [0] errcode: 错误码
            [1] inpos: 是否到位(0/1)
            [2] powered_on: 是否上电(0/1)
            [3] enabled: 是否使能(0/1)
            [4] rapidrate: 运行倍率
            [5] protective_stop: 碰撞检测(0/1)
            [6] drag_status: 拖拽状态(0/1)
            [7] on_soft_limit: 限位状态(0/1)
            [8] current_user_id: 当前用户坐标系ID
            [9] current_tool_id: 当前工具坐标系ID
            [10] dout: 控制柜数字输出
            [11] din: 控制柜数字输入
            [12] aout: 控制柜模拟输出
            [13] ain: 控制柜模拟输入
            [14] tio_dout: 末端数字输出
            [15] tio_din: 末端数字输入
            [16] tio_ain: 末端模拟输入
            [17] extio: 扩展IO信号
            [18] cart_position: 笛卡尔位置
            [19] joint_position: 关节位置
            [20] robot_monitor_data: 机器人监测数据
            [21] torq_sensor_monitor_data: 力矩传感器数据
            [22] is_socket_connect: 连接状态(0/1)
            [23] emergency_stop: 急停状态(0/1)
            [24] tio_key: 末端按钮状态
        """
        return self.robot.get_robot_status()
    
    def set_status_data_update_time_interval(self, millisecond: int) -> Tuple[int, ...]:
        """
        设置状态数据自动更新时间间隔
        
        Args:
            millisecond: 更新间隔,单位ms,默认4ms
            
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.set_status_data_update_time_interval(millisecond)
    
    def get_joint_position(self) -> Tuple[int, List[float]]:
        """
        获取当前关节角度
        
        Returns:
            (0, [j1,j2,j3,j4,j5,j6]): 成功,单位rad
            其他: 失败
        """
        return self.robot.get_joint_position()
    
    def get_tcp_position(self) -> Tuple[int, List[float]]:
        """
        获取当前TCP位姿
        
        Returns:
            (0, [x,y,z,rx,ry,rz]): 成功,位置单位mm,姿态单位rad
            其他: 失败
        """
        return self.robot.get_tcp_position()
    
    def get_robot_state(self) -> Tuple[int, Tuple[int, int, int]]:
        """
        获取机械臂基本状态
        
        Returns:
            (0, (estoped, power_on, servo_enabled)): 成功
            estoped: 急停状态 0=未按下, 1=按下
            power_on: 上电状态 0=未上电, 1=已上电
            servo_enabled: 使能状态 0=未使能, 1=已使能
        """
        return self.robot.get_robot_state()
