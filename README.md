# JAKA Visual Servo (Track Red Cube)

这是一个基于 Intel RealSense 相机与 JAKA 机械臂的视觉伺服演示项目，用于检测红色正方形并驱动 JAKA 机械臂抓取/跟踪。

## 目录

- `MyTrackRedCubeServo.py` - 主运行脚本（入口）
- `jaka_robot/` - JAKA 机械臂控制封装模块
- `__common.py` - 项目级共享设置（请按需检查）
- `CONTRIBUTING.md` - 贡献指南

## 特性

- 使用 RealSense 获取彩色图像
- 基于 OpenCV 检测红色正方形并计算质心
- 将像素误差转换为机器人运动命令
- 封装的 JAKA 控制器（`jaka_robot`），调用 `jkrc` 库与硬件通信

## 前提条件

- Windows 操作系统（此仓库中使用了 `msvcrt` 等 Windows 特性）
- Python 3.8+
- Intel RealSense 相机与对应 SDK（pyrealsense2）
- JAKA 机器人 SDK（`jkrc` 模块）——此模块通常由 JAKA 厂商提供并单独安装，可能不是 pip 包，请按照厂商说明安装并将其放在 Python 可导入路径中

## 安装依赖

推荐使用虚拟环境（venv 或 conda）。在项目根目录下运行：

```pwsh
# 创建并激活 venv（PowerShell）
python -m venv .venv
.\.venv\Scripts\Activate.ps1

# 安装依赖
pip install -r requirements.txt
```

注意：
- `jkrc`（JAKA SDK）通常不在 pip 上，需要按厂商说明安装。
- RealSense 的驱动/固件需要事先安装（特别是在 Windows 上），并确保 `pyrealsense2` 与相机固件兼容。

## 运行

编辑代码中的配置（如机器人 IP、相机参数等），然后运行：

```pwsh
python MyTrackRedCubeServo.py
```

常见运行模式：
- 直接在带显示器的 PC 上运行以查看调试窗口（需要显示环境）
- 在 headless 环境下运行时请关闭任何显示/保存调试图像的选项

## 配置

`MyTrackRedCubeServo.py` 包含若干配置类：
- `RobotConfig`：机器人 IP、HOME 位姿等
- `CameraConfig`：分辨率、帧率、像素格式
- `ControlConfig`：控制增益、PID 参数和伺服参数
- `VisionConfig`：HSV 红色阈值、形态学与最小轮廓面积

根据你的机器人和视觉场景微调这些参数以获得最佳效果。

## 开发与贡献

欢迎提交 PR 或 issue。请参阅项目根目录下的 `CONTRIBUTING.md` 了解贡献规范。

## 许可

本仓库包含 `LICENSE` 文件，请根据 LICENSE 内容使用/分发代码。

---