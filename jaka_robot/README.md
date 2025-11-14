# JAKA机械臂模块化控制包

## 📦 版本信息
- **版本**: 2.0.0 (模块化版本)
- **基于**: JAKA Python SDK V2.1.11
- **作者**: AI Assistant
- **日期**: 2025年10月

## 🎯 模块化设计目标

将原来单一的大型控制类拆分为多个功能模块，使代码：
- ✅ 更易维护和理解
- ✅ 更便于扩展新功能
- ✅ 更好的代码组织结构
- ✅ 支持按需导入模块

## 📁 目录结构

```
jaka_robot/
├── __init__.py          # 包初始化，导出主控制器类
├── constants.py         # 常量定义（IO类型、坐标系类型等）
├── base.py             # 基础控制（登录、上电、使能等）
├── motion.py           # 运动控制（关节运动、直线运动、伺服等）
├── io_control.py       # IO控制（数字IO、模拟IO）
├── coordinate.py       # 坐标系管理（用户坐标系、工具坐标系、运动学）
├── safety.py           # 安全管理（碰撞检测、限位、错误处理）
├── status.py           # 状态查询（机器人状态、位置信息）
├── program.py          # 程序控制（加载、运行、暂停程序）
└── utils.py            # 工具函数（初始化、关闭、等待等便捷方法）
```

## 🚀 快速开始

### 1. 基础使用

```python
import __common
from jaka_robot import JAKARobotController, RobotConstants

# 初始化环境
__common.init_env()
import jkrc

# 创建控制器
robot = JAKARobotController("10.5.5.100")

# 初始化机器人
robot.initialize_robot()

# 执行运动
target = [0.5, 0.5, 0, 0, 0, 0]
robot.motion.joint_move(target, RobotConstants.ABS, True, 0.5)

# 安全关闭
robot.safe_shutdown()
```

### 2. 模块化使用

```python
# 使用base模块
robot.base.login()
robot.base.power_on()
robot.base.enable_robot()

# 使用motion模块
robot.motion.joint_move([0,0,0,0,0,0], RobotConstants.ABS, True, 0.5)
robot.motion.linear_move([0,0,50,0,0,0], RobotConstants.INCR, True, 50)

# 使用io模块
robot.io.set_digital_output(RobotConstants.IO_CABINET, 0, 1)
value = robot.io.get_digital_input(RobotConstants.IO_CABINET, 0)

# 使用status模块
joint_pos = robot.status.get_joint_position()
tcp_pos = robot.status.get_tcp_position()

# 使用coordinate模块
robot.coordinate.set_tool_data(1, [0,0,100,0,0,0], "MyTool")
robot.coordinate.set_user_frame_id(1)

# 使用safety模块
robot.safety.set_collision_level(3)
is_collision = robot.safety.is_in_collision()

# 使用program模块
robot.program.set_rapidrate(0.5)
robot.program.program_load("my_program")
```

### 3. 使用工具函数

```python
# 便捷的初始化和关闭
robot.initialize_robot(wait_time=8.0)
robot.safe_shutdown()

# 检查机器人状态
if robot.check_robot_ready():
    print("机器人已就绪")

# 获取完整位姿信息
pose_info = robot.get_current_pose_info()
print(pose_info['joint_deg'])  # 关节角度（度）
print(pose_info['tcp_pos'])    # TCP位姿

# 打印状态摘要
robot.print_status_summary()

# 等待运动完成
robot.wait_motion_done(timeout=30)

# 移动到HOME位置
robot.move_to_home()
```

## 📚 模块详解

### constants.py - 常量定义
定义所有机器人控制相关的常量：
- IO类型：`IO_CABINET`, `IO_TOOL`, `IO_EXTEND`
- 坐标系类型：`COORD_BASE`, `COORD_JOINT`, `COORD_TOOL`
- 运动类型：`ABS` (绝对), `INCR` (增量)
- 碰撞等级：`COLLISION_LEVEL_1` ~ `COLLISION_LEVEL_5`
- 角度转换：`DEG_TO_RAD`, `RAD_TO_DEG`

### base.py - 基础控制
机器人基本操作：
- `login()` / `logout()` - 登录/登出
- `power_on()` / `power_off()` - 上电/断电
- `enable_robot()` / `disable_robot()` - 使能/下使能
- `drag_mode_enable()` - 拖拽模式
- `set_payload()` - 设置负载

### motion.py - 运动控制
各种运动功能：
- `joint_move()` - 关节空间运动
- `linear_move()` - 直线运动
- `circular_move()` - 圆弧运动
- `servo_j()` / `servo_p()` - 伺服运动
- `motion_abort()` - 终止运动
- `is_in_pos()` - 查询是否到位

### io_control.py - IO控制
数字和模拟IO操作：
- `set_digital_output()` / `get_digital_input()`
- `set_analog_output()` / `get_analog_input()`
- `is_extio_running()` - 查询扩展IO状态

### coordinate.py - 坐标系管理
坐标系设置和运动学计算：
- `set_user_frame_data()` - 设置用户坐标系
- `set_tool_data()` - 设置工具坐标系
- `kine_forward()` - 正解
- `kine_inverse()` - 逆解
- `rpy_to_rot_matrix()` - 欧拉角转旋转矩阵

### safety.py - 安全管理
安全相关功能：
- `set_collision_level()` - 设置碰撞等级
- `is_in_collision()` - 查询碰撞状态
- `collision_recover()` - 碰撞恢复
- `is_on_limit()` - 查询限位
- `clear_error()` - 清除错误

### status.py - 状态查询
机器人状态获取：
- `get_robot_status()` - 获取完整状态
- `get_joint_position()` - 获取关节位置
- `get_tcp_position()` - 获取TCP位置
- `get_robot_state()` - 获取基本状态

### program.py - 程序控制
程序加载和执行：
- `program_load()` / `program_run()`
- `program_pause()` / `program_resume()`
- `program_abort()` - 终止程序
- `set_rapidrate()` - 设置速度倍率

### utils.py - 工具函数
便捷的高级功能：
- `initialize_robot()` - 完整初始化流程
- `safe_shutdown()` - 安全关闭流程
- `move_to_home()` - 移动到HOME位置
- `wait_motion_done()` - 等待运动完成
- `check_robot_ready()` - 检查就绪状态
- `get_current_pose_info()` - 获取位姿信息
- `print_status_summary()` - 打印状态摘要

## 🔧 与原版本的兼容性

模块化版本保持了与原版本类似的API，但组织方式更清晰：

**原版本:**
```python
robot = JAKARobotController("10.5.5.100")
robot.login()
robot.power_on()
robot.joint_move(...)
robot.set_digital_output(...)
```

**新版本:**
```python
robot = JAKARobotController("10.5.5.100")
robot.base.login()
robot.base.power_on()
robot.motion.joint_move(...)
robot.io.set_digital_output(...)
```

**便捷方法 (直接调用):**
```python
robot.initialize_robot()  # 代替 login + power_on + enable
robot.safe_shutdown()      # 代替 disable + power_off + logout
```

## 📝 示例文件

查看 `jaka_modular_examples.py` 获取详细使用示例，包括：
1. 基础使用示例
2. 模块化控制示例
3. IO控制示例
4. 高级运动控制示例
5. 程序控制示例
6. 工具函数示例

运行示例：
```bash
python jaka_modular_examples.py
```

## ⚠️ 注意事项

1. **导入顺序**: 必须先调用 `__common.init_env()` 再导入 `jkrc`
2. **初始化**: 使用 `initialize_robot()` 可自动完成登录、上电、使能
3. **关闭**: 始终使用 `safe_shutdown()` 安全关闭机器人
4. **错误处理**: 所有函数返回元组，第一个元素为错误码(0表示成功)
5. **阻塞模式**: 阻塞运动会等待完成，非阻塞需手动等待

## 🎓 优势

### 代码组织
- 每个模块专注于特定功能领域
- 减少单个文件的复杂度
- 便于理解和维护

### 可扩展性
- 新功能可以独立添加到相应模块
- 不影响其他模块的功能
- 易于版本控制和协作开发

### 可测试性
- 每个模块可以独立测试
- 便于编写单元测试
- 减少模块间的耦合

### 使用灵活性
- 可以按需导入特定模块
- 支持直接访问低级API
- 提供高级便捷函数

## 📄 许可证

基于JAKA Python SDK开发，遵循相应的许可协议。

## 🔗 相关资源

- JAKA官方文档
- Python SDK手册
- API参考文档

---

**版本历史:**
- v2.0.0 (2025-10): 模块化重构
- v1.0.0: 初始版本（单文件版本）
