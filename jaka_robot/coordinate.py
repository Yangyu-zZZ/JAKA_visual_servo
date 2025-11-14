"""
JAKA机械臂坐标系管理模块
包含用户坐标系、工具坐标系设置及运动学计算
"""

from typing import List, Tuple


class CoordinateSystem:
    """机器人坐标系管理功能"""
    
    def __init__(self, robot):
        """
        初始化坐标系管理模块
        
        Args:
            robot: JAKA机器人对象(jkrc.RC实例)
        """
        self.robot = robot
    
    # ==================== 用户坐标系 ====================
    
    def set_user_frame_data(self, frame_id: int, user_frame: List[float], 
                           name: str = "") -> Tuple[int, ...]:
        """
        设置用户坐标系
        
        Args:
            frame_id: 坐标系ID(1-10), 0为基坐标系
            user_frame: 坐标系参数[x,y,z,rx,ry,rz]
            name: 坐标系别名
            
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.set_user_frame_data(frame_id, user_frame, name)
    
    def get_user_frame_data(self, frame_id: int) -> Tuple[int, int, List[float]]:
        """
        获取用户坐标系数据
        
        Args:
            frame_id: 坐标系ID
            
        Returns:
            (0, id, [x,y,z,rx,ry,rz]): 成功
            其他: 失败
        """
        return self.robot.get_user_frame_data(frame_id)
    
    def get_user_frame_id(self) -> Tuple[int, int]:
        """
        获取当前用户坐标系ID
        
        Returns:
            (0, id): 成功, id范围0-10
            其他: 失败
        """
        return self.robot.get_user_frame_id()
    
    def set_user_frame_id(self, frame_id: int) -> Tuple[int, ...]:
        """
        设置当前用户坐标系ID
        
        Args:
            frame_id: 坐标系ID(0-10)
            
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.set_user_frame_id(frame_id)
    
    # ==================== 工具坐标系 ====================
    
    def set_tool_data(self, tool_id: int, tcp: List[float], 
                     name: str = "") -> Tuple[int, ...]:
        """
        设置工具坐标系
        
        Args:
            tool_id: 工具ID(1-10), 0为末端法兰
            tcp: 工具参数[x,y,z,rx,ry,rz]
            name: 工具别名
            
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.set_tool_data(tool_id, tcp, name)
    
    def get_tool_data(self, tool_id: int) -> Tuple[int, int, List[float]]:
        """
        获取工具坐标系数据
        
        Args:
            tool_id: 工具ID
            
        Returns:
            (0, id, [x,y,z,rx,ry,rz]): 成功
            其他: 失败
        """
        return self.robot.get_tool_data(tool_id)
    
    def get_tool_id(self) -> Tuple[int, int]:
        """
        获取当前工具ID
        
        Returns:
            (0, id): 成功, id范围0-10
            其他: 失败
        """
        return self.robot.get_tool_id()
    
    def set_tool_id(self, tool_id: int) -> Tuple[int, ...]:
        """
        设置当前工具ID
        
        Args:
            tool_id: 工具ID(0-10)
            
        Returns:
            (0,): 成功
            其他: 失败
        """
        return self.robot.set_tool_id(tool_id)
    
    # ==================== 运动学计算 ====================
    
    def kine_forward(self, joint_pos: List[float]) -> Tuple[int, List[float]]:
        """
        机器人正解(关节角→笛卡尔位姿)
        
        Args:
            joint_pos: 关节角度[j1,j2,j3,j4,j5,j6],单位rad
            
        Returns:
            (0, [x,y,z,rx,ry,rz]): 成功
            其他: 失败
        """
        return self.robot.kine_forward(joint_pos)
    
    def kine_inverse(self, ref_pos: List[float], 
                    cartesian_pose: List[float]) -> Tuple[int, List[float]]:
        """
        机器人逆解(笛卡尔位姿→关节角)
        
        Args:
            ref_pos: 参考关节位置(建议使用当前位置)
            cartesian_pose: 目标笛卡尔位姿[x,y,z,rx,ry,rz]
            
        Returns:
            (0, [j1,j2,j3,j4,j5,j6]): 成功,单位rad
            (-4,): 逆解失败
            其他: 其他错误
        """
        return self.robot.kine_inverse(ref_pos, cartesian_pose)
    
    def rpy_to_rot_matrix(self, rpy: List[float]) -> Tuple[int, List[List[float]]]:
        """
        欧拉角转旋转矩阵
        
        Args:
            rpy: 欧拉角[rx,ry,rz],单位rad
            
        Returns:
            (0, rot_matrix): 成功, 3x3旋转矩阵
            其他: 失败
        """
        return self.robot.rpy_to_rot_matrix(rpy)
    
    def rot_matrix_to_rpy(self, rot_matrix: List[List[float]]) -> Tuple[int, List[float]]:
        """
        旋转矩阵转欧拉角
        
        Args:
            rot_matrix: 3x3旋转矩阵
            
        Returns:
            (0, [rx,ry,rz]): 成功
            其他: 失败
        """
        return self.robot.rot_matrix_to_rpy(rot_matrix)
    
    def quaternion_to_rot_matrix(self, quaternion: List[float]) -> Tuple[int, List[List[float]]]:
        """
        四元数转旋转矩阵
        
        Args:
            quaternion: 四元数[w,x,y,z]
            
        Returns:
            (0, rot_matrix): 成功, 3x3旋转矩阵
            其他: 失败
        """
        return self.robot.quaternion_to_rot_matrix(quaternion)
    
    def rot_matrix_to_quaternion(self, rot_matrix: List[List[float]]) -> Tuple[int, List[float]]:
        """
        旋转矩阵转四元数
        
        Args:
            rot_matrix: 3x3旋转矩阵
            
        Returns:
            (0, [w,x,y,z]): 成功
            其他: 失败
        """
        return self.robot.rot_matrix_to_quaternion(rot_matrix)
