"""
JAKA机械臂工具函数模块
包含初始化、关闭、等待运动完成等便捷函数
"""

import time
from typing import Dict, Any, List
from .constants import RobotConstants


class RobotUtils:
    """机器人工具函数集"""
    
    def __init__(self, controller):
        """
        初始化工具函数模块
        
        Args:
            controller: JAKARobotController实例
        """
        self.controller = controller
    
    def initialize_robot(self, wait_time: float = RobotConstants.DEFAULT_POWERUP_TIME) -> bool:
        """
        机器人完整初始化流程(登录→上电→使能)
        
        Args:
            wait_time: 上电等待时间(秒),默认8秒
            
        Returns:
            True: 初始化成功
            False: 初始化失败
        """
        # 登录
        ret = self.controller.base.login()
        if ret[0] != 0:
            print(f"登录失败: 错误码 {ret[0]}")
            return False
        print("✓ 机器人登录成功")
        
        # 上电
        ret = self.controller.base.power_on()
        if ret[0] != 0:
            print(f"上电失败: 错误码 {ret[0]}")
            return False
        print(f"✓ 机器人上电成功,等待{wait_time}秒...")
        time.sleep(wait_time)
        
        # 使能
        ret = self.controller.base.enable_robot()
        if ret[0] != 0:
            print(f"使能失败: 错误码 {ret[0]}")
            return False
        print("✓ 机器人使能成功")
        
        # 检查状态
        ret = self.controller.status.get_robot_status()
        if ret[0] == 0:
            status = ret[1]
            print(f"机器人状态: 上电={status[2]}, 使能={status[3]}, 急停={status[23]}")
            if status[2] == 1 and status[3] == 1 and status[23] == 0:
                print("✓ 机器人初始化完成,可以开始运动")
                return True
        
        print("✗ 机器人状态异常")
        return False
    
    def safe_shutdown(self, wait_time: float = 8.0) -> bool:
        """
        机器人安全关闭流程(下使能→断电→登出)
        
        Args:
            wait_time: 下使能后等待时间(秒)
            
        Returns:
            True: 关闭成功
            False: 关闭失败
        """
        success = True
        
        # 下使能
        ret = self.controller.base.disable_robot()
        if ret[0] != 0:
            print(f"下使能失败: 错误码 {ret[0]}")
            success = False
        else:
            print(f"✓ 机器人下使能成功,等待{wait_time}秒...")
            time.sleep(wait_time)
        
        # 断电
        ret = self.controller.base.power_off()
        if ret[0] != 0:
            print(f"断电失败: 错误码 {ret[0]}")
            success = False
        else:
            print("✓ 机器人断电成功")
        
        # 登出
        ret = self.controller.base.logout()
        if ret[0] != 0:
            print(f"登出失败: 错误码 {ret[0]}")
            success = False
        else:
            print("✓ 机器人登出成功")
        
        return success
    
    def move_to_home(self, home_joint: List[float] = None, 
                    speed: float = RobotConstants.DEFAULT_JOINT_SPEED) -> bool:
        """
        移动到HOME位置
        
        Args:
            home_joint: HOME关节角度,默认为全0位置
            speed: 运动速度,单位rad/s
            
        Returns:
            True: 成功
            False: 失败
        """
        if home_joint is None:
            home_joint = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        ret = self.controller.motion.joint_move(home_joint, RobotConstants.ABS, True, speed)
        if ret[0] == 0:
            print("✓ 已移动到HOME位置")
            return True
        else:
            print(f"✗ 移动失败: 错误码 {ret[0]}")
            return False
    
    def wait_motion_done(self, timeout: float = RobotConstants.DEFAULT_TIMEOUT, 
                        check_interval: float = RobotConstants.DEFAULT_CHECK_INTERVAL) -> bool:
        """
        等待运动完成
        
        Args:
            timeout: 超时时间(秒)
            check_interval: 检查间隔(秒)
            
        Returns:
            True: 运动完成
            False: 超时
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            ret = self.controller.motion.is_in_pos()
            if ret[0] == 0 and ret[1] == 1:
                return True
            time.sleep(check_interval)
        
        print(f"✗ 等待运动完成超时({timeout}秒)")
        return False
    
    def check_robot_ready(self) -> bool:
        """
        检查机器人是否就绪(上电、使能、无急停)
        
        Returns:
            True: 就绪
            False: 未就绪
        """
        ret = self.controller.status.get_robot_status()
        if ret[0] != 0:
            print("✗ 无法获取机器人状态")
            return False
        
        status = ret[1]
        powered = status[2]
        enabled = status[3]
        emergency = status[23]
        
        if powered != 1:
            print("✗ 机器人未上电")
            return False
        if enabled != 1:
            print("✗ 机器人未使能")
            return False
        if emergency == 1:
            print("✗ 急停按钮被按下")
            return False
        
        return True
    
    def get_current_pose_info(self) -> Dict[str, Any]:
        """
        获取当前位姿完整信息
        
        Returns:
            字典包含:
            - joint_pos: 关节角度(rad)
            - tcp_pos: TCP位姿(mm, rad)
            - joint_deg: 关节角度(度)
            - tcp_pos_deg: TCP位姿(mm, 度)
        """
        info = {}
        
        # 获取关节位置
        ret = self.controller.status.get_joint_position()
        if ret[0] == 0:
            info['joint_pos'] = ret[1]
            info['joint_deg'] = [j * RobotConstants.RAD_TO_DEG for j in ret[1]]
        
        # 获取TCP位置
        ret = self.controller.status.get_tcp_position()
        if ret[0] == 0:
            tcp = ret[1]
            info['tcp_pos'] = tcp
            info['tcp_pos_deg'] = tcp[:3] + [r * RobotConstants.RAD_TO_DEG for r in tcp[3:]]
        
        return info
    
    def print_status_summary(self):
        """打印机器人状态摘要"""
        ret = self.controller.status.get_robot_status()
        if ret[0] != 0:
            print("✗ 无法获取机器人状态")
            return
        
        status = ret[1]
        print("\n" + "="*50)
        print("机器人状态摘要".center(48))
        print("="*50)
        print(f"错误码: {status[0]}")
        print(f"到位状态: {'已到位' if status[1] == 1 else '运动中'}")
        print(f"上电状态: {'已上电' if status[2] == 1 else '未上电'}")
        print(f"使能状态: {'已使能' if status[3] == 1 else '未使能'}")
        print(f"运行倍率: {status[4]}")
        print(f"碰撞检测: {'碰撞' if status[5] == 1 else '正常'}")
        print(f"拖拽模式: {'开启' if status[6] == 1 else '关闭'}")
        print(f"限位状态: {'超限' if status[7] == 1 else '正常'}")
        print(f"用户坐标系ID: {status[8]}")
        print(f"工具坐标系ID: {status[9]}")
        print(f"连接状态: {'正常' if status[22] == 1 else '异常'}")
        print(f"急停状态: {'按下' if status[23] == 1 else '释放'}")
        print("="*50 + "\n")

    def print_pose_info(self):
        """打印机器人位姿信息

        输出内容包含关节角（rad & deg）和TCP位姿（mm & deg），如果无法获取则打印错误信息。
        """
        info = self.get_current_pose_info()

        if not info:
            print("✗ 无法获取位姿信息")
            return

        joint_pos = info.get('joint_pos')
        joint_deg = info.get('joint_deg')
        tcp_pos = info.get('tcp_pos')
        tcp_pos_deg = info.get('tcp_pos_deg')

        print('\n' + '-'*50)
        print('机器人位姿信息'.center(48))
        print('-'*50)

        if joint_pos is not None:
            print('关节角 (rad):')
            print('  ' + ', '.join(f"J{i+1}: {p:.6f}" for i, p in enumerate(joint_pos)))
            if joint_deg is not None:
                print('关节角 (deg):')
                print('  ' + ', '.join(f"J{i+1}: {d:.3f}°" for i, d in enumerate(joint_deg)))
        else:
            print('✗ 无法获取关节角')

        if tcp_pos is not None:
            # tcp_pos assumed as [x, y, z, rx, ry, rz]
            print('\nTCP 位姿 (mm, rad):')
            print('  X: {0:.3f} mm, Y: {1:.3f} mm, Z: {2:.3f} mm'.format(*tcp_pos[:3]))
            print('  Rx: {0:.6f} rad, Ry: {1:.6f} rad, Rz: {2:.6f} rad'.format(*tcp_pos[3:]))
            if tcp_pos_deg is not None:
                print('\nTCP 位姿 (mm, deg):')
                print('  X: {0:.3f} mm, Y: {1:.3f} mm, Z: {2:.3f} mm'.format(*tcp_pos_deg[:3]))
                print('  Rx: {0:.3f}°, Ry: {1:.3f}°, Rz: {2:.3f}°'.format(*tcp_pos_deg[3:]))
        else:
            print('✗ 无法获取TCP位姿')

        print('-'*50 + '\n')
