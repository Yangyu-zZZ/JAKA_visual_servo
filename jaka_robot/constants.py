"""
JAKA机械臂常量定义模块
包含所有机器人控制相关的常量定义
"""

import math
import numpy as np


class RobotConstants:
    """机器人控制常量"""
    
    # IO类型
    IO_CABINET = 0  # 控制柜面板IO
    IO_TOOL = 1     # 工具IO
    IO_EXTEND = 2   # 扩展IO
    
    # 坐标系类型
    COORD_BASE = 0   # 基坐标系/当前用户坐标系
    COORD_JOINT = 1  # 关节空间
    COORD_TOOL = 2   # 工具坐标系
    
    # 运动类型
    ABS = 0   # 绝对运动
    INCR = 1  # 增量运动
    
    # 角度/弧度转换
    PI = math.pi
    DEG_TO_RAD = PI / 180.0
    RAD_TO_DEG = 180.0 / PI
    
    # 运动模式
    MOVE_MODE_ABS = 0      # 绝对运动
    MOVE_MODE_INCR = 1     # 增量运动
    MOVE_MODE_CONTINUOUS = 2  # 连续运动
    
    # 程序状态
    PROGRAM_STOP = 0    # 程序停止
    PROGRAM_RUNNING = 1 # 程序运行中
    PROGRAM_PAUSE = 2   # 程序暂停
    
    # 碰撞等级
    COLLISION_OFF = 0      # 关闭碰撞检测
    COLLISION_LEVEL_1 = 1  # 25N阈值
    COLLISION_LEVEL_2 = 2  # 50N阈值
    COLLISION_LEVEL_3 = 3  # 75N阈值
    COLLISION_LEVEL_4 = 4  # 100N阈值
    COLLISION_LEVEL_5 = 5  # 125N阈值
    
    # 网络异常处理动作
    NET_EXCEPTION_KEEP = 0     # 保持原运动
    NET_EXCEPTION_PAUSE = 1    # 暂停运动
    NET_EXCEPTION_ABORT = 2    # 终止运动
    
    # 力控类型
    COMPLIANCE_OFF = 0       # 不使用柔顺控制
    COMPLIANCE_FORCE = 1     # 恒力柔顺控制
    COMPLIANCE_VELOCITY = 2  # 速度柔顺控制
    
    # 力矩传感器品牌
    SENSOR_SONY = 1        # SONY索尼半导体
    SENSOR_BOSCH = 2       # BoschSensortec博世
    SENSOR_ST = 3          # ST意法半导体
    
    # FTP操作类型
    FTP_FILE = 1      # 单个文件
    FTP_FOLDER = 2    # 文件夹
    
    # FTP目录类型
    FTP_DIR_ALL = 0     # 文件和文件夹
    FTP_DIR_FILE = 1    # 仅文件
    FTP_DIR_FOLDER = 2  # 仅文件夹
    
    # RS485通道模式
    RS485_MODBUS_RTU = 0  # Modbus RTU
    RS485_RAW = 1         # Raw RS485
    RS485_TORQUE = 2      # 力矩传感器
    
    # TIO引脚类型
    TIO_PIN_DI = 0  # 数字输入
    TIO_PIN_DO = 1  # 数字输出
    TIO_PIN_AI = 2  # 模拟输入
    
    # 默认值
    DEFAULT_JOINT_SPEED = 0.5      # 默认关节速度 (rad/s)
    DEFAULT_LINEAR_SPEED = 50.0    # 默认直线速度 (mm/s)
    DEFAULT_ACCELERATION = 0.5     # 默认加速度
    DEFAULT_TOLERANCE = 0.01       # 默认终点误差
    DEFAULT_POWERUP_TIME = 8.0     # 默认上电等待时间 (秒)
    DEFAULT_CHECK_INTERVAL = 0.1   # 默认检查间隔 (秒)
    DEFAULT_TIMEOUT = 30.0         # 默认超时时间 (秒)


# Eye-to-hand 标定矩阵: 摄像头(eye)到机械臂基座(hand/base)的变换矩阵 眼在手上
# 单位：平移以毫米(mm)为单位
    camera_to_TCP = np.array([
        [-9.96540760e-01, -7.57316262e-02, 3.42233147e-02, 2.57855395e+01],
        [7.35189897e-02, -9.95374204e-01, -6.18478161e-02, 9.45864984e+01],
        [3.87488403e-02, -5.91178061e-02, 9.97498678e-01, 2.77075673e+01],
        [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
    ], dtype=np.float64)
