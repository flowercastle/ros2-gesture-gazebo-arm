# 基于视觉手势识别的Gazebo虚拟机械臂交互控制系统使用说明

## 1.项目简介

本项目实现了一个面向人机交互课程展示的虚拟机械臂控制系统。用户通过摄像头做出手势，系统使用MediaPipe识别手部关键点，将稳定手势映射为ROS2关节轨迹指令，并在Gazebo中驱动虚拟机械臂运动。

项目重点不是复杂路径规划，而是完成一个稳定、可展示、可解释的人机交互闭环：

```text
摄像头手势输入 → 手势识别与防抖 → ROS2控制节点 → Gazebo机械臂运动 → 屏幕/Gazebo实时反馈
```

## 2.环境说明

推荐环境：

```text
Ubuntu24.04
ROS2 Jazzy
Gazebo Harmonic/Gazebo Sim
colcon
ament_python
gz_ros2_control
joint_trajectory_controller
OpenCV
MediaPipe
Pillow
```

安装常用依赖：

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-pip \
  python3-opencv \
  python3-pil \
  fonts-noto-cjk \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-xacro
```

MediaPipe兼容版本建议：

```bash
python3 -m pip install --user --break-system-packages --force-reinstall \
  "numpy==1.26.4" \
  "opencv-contrib-python==4.10.0.84" \
  "protobuf<5" \
  "mediapipe==0.10.21"
```

测试MediaPipe是否可用：

```bash
python3 - <<'PY'
import numpy
import cv2
import mediapipe as mp
print("numpy:", numpy.__version__)
print("opencv:", cv2.__version__)
print("mediapipe:", mp.__version__)
print("has solutions:", hasattr(mp, "solutions"))
print("hands:", mp.solutions.hands.Hands)
PY
```

## 3.获取源码

```bash
mkdir -p ~/gesture_ws/src
cd ~/gesture_ws/src

git clone https://github.com/<flowercastle>/ros2-gesture-gazebo-arm.git

cd ~/gesture_ws
colcon build --symlink-install
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## 4.工程结构

```text
~/hci_ws/src/gesture_gazebo_arm
├── config
│   └── controllers.yaml
├── launch
│   └── demo.launch.py
├── urdf
│   └── gesture_arm.urdf.xacro
├── package.xml
├── setup.py
└── gesture_gazebo_arm
    ├── __init__.py
    ├── gesture_arm_controller.py
    └── test_arm_controller.py
```

核心文件说明：

| 文件 | 作用 |
|---|---|
| `gesture_arm.urdf.xacro` | 定义Gazebo机械臂模型、关节、ros2_control接口和插件 |
| `controllers.yaml` | 配置`joint_state_broadcaster`和`arm_controller` |
| `demo.launch.py` | 启动Gazebo、发布robot_description、生成模型、加载控制器 |
| `test_arm_controller.py` | 不依赖摄像头，用固定轨迹测试机械臂是否能动 |
| `gesture_arm_controller.py` | 摄像头手势识别和机械臂实时控制主程序 |

## 5.编译项目

```bash
cd ~/hci_ws
colcon build --symlink-install
source /opt/ros/jazzy/setup.bash
source ~/hci_ws/install/setup.bash
```

如果修改了Python脚本或URDF，建议重新执行：

```bash
cd ~/hci_ws
colcon build --symlink-install
source install/setup.bash
```

## 6.启动前清理残留进程

如果重复启动后出现两个机械臂，或Gazebo状态异常，先清理残留进程：

```bash
pkill -f gz
pkill -f gazebo
pkill -f ros_gz_sim
pkill -f controller_manager
pkill -f robot_state_publisher
pkill -f spawner
```

注意：`demo.launch.py`中建议删除`-allow_renaming true`，否则重复启动时Gazebo可能自动生成`gesture_arm_0`，导致画面里出现多个机械臂。

## 7.运行流程

### 7.1启动Gazebo机械臂

终端1：

```bash
source /opt/ros/jazzy/setup.bash
source ~/hci_ws/install/setup.bash
ros2 launch gesture_gazebo_arm demo.launch.py
```

启动后应看到Gazebo窗口和一个虚拟机械臂。

### 7.2检查控制器状态

终端2：

```bash
source /opt/ros/jazzy/setup.bash
source ~/hci_ws/install/setup.bash
ros2 control list_controllers
```

理想输出中应包含：

```text
joint_state_broadcaster active
arm_controller active
```

### 7.3测试机械臂控制链路

终端2继续执行：

```bash
ros2 run gesture_gazebo_arm test_arm_controller
```

如果机械臂能自动运动，说明Gazebo、ros2_control、控制器和话题链路都正常。

停止测试脚本：

```bash
Ctrl+C
```

### 7.4运行手势控制节点

终端2执行：

```bash
ros2 run gesture_gazebo_arm gesture_arm_controller
```

程序会自动扫描`/dev/video0~7`，找到可用摄像头后弹出窗口。窗口中会显示：

```text
当前摄像头
当前识别
稳定指令
操作说明
```

## 8.手势映射表

| 手势 | 系统识别结果 | 机械臂动作 |
|---|---|---|
| 握拳 | `FIST_STOP` | 停止，不发送新动作 |
| 张开手掌 | `PALM_HOME` | 回到初始姿态 |
| 单指向左 | `POINT_LEFT` | 底座左转 |
| 单指向右 | `POINT_RIGHT` | 底座右转 |
| 单指向上 | `POINT_UP` | 大臂上抬 |
| 单指向下 | `POINT_DOWN` | 大臂下压 |
| 两指 | `TWO_ELBOW_UP` | 小臂上抬 |
| 三指 | `THREE_ELBOW_DOWN` | 小臂下压 |
| OK手势 | `OK_CLOSE` | 夹爪闭合 |

## 9.交互稳定性设计

本项目对手势识别做了几项稳定性处理：

1.自动扫描摄像头编号，避免固定`VideoCapture(0)`导致打不开摄像头。
2.使用Pillow绘制中文文字，解决OpenCV中文显示问题。
3.使用7帧投票防抖，最近7帧中至少4帧一致才更新稳定手势。
4.单指方向判断使用“指尖远离手腕/掌心”的方向无关规则，避免单指向下被误判为握拳。
5.所有关节角度都做限幅，避免指令超出URDF关节范围。
6.URDF中关闭各link重力，使机械臂只响应手势控制，更适合HCI课堂展示。

## 10.常见问题与解决方法

### 10.1 Gazebo里出现两个机械臂

原因：Gazebo进程没有完全关闭，或者`demo.launch.py`中保留了`-allow_renaming true`。

解决：

```bash
pkill -f gz
pkill -f gazebo
pkill -f ros_gz_sim
```

并删除`demo.launch.py`里的：

```python
"-allow_renaming", "true"
```

### 10.2 摄像头窗口打不开

检查设备：

```bash
ls /dev/video*
```

安装查看工具：

```bash
sudo apt install -y v4l-utils
v4l2-ctl --list-devices
```

新版`gesture_arm_controller.py`会自动扫描`/dev/video0~7`。

### 10.3 `mediapipe`没有`solutions`

原因：MediaPipe版本太新。

解决：安装兼容旧接口的版本：

```bash
python3 -m pip install --user --break-system-packages --force-reinstall \
  "numpy==1.26.4" \
  "opencv-contrib-python==4.10.0.84" \
  "protobuf<5" \
  "mediapipe==0.10.21"
```

### 10.4 单指向下被识别为握拳

原因：旧逻辑只用`tip.y < pip.y`判断手指伸直，只适合手指向上。

解决：使用最新版`gesture_arm_controller.py`，已经改为方向无关判断。

### 10.5 机械臂不控制也自己倾斜

原因：水平悬臂结构在Gazebo物理仿真中受到重力和控制器初始化影响。

解决：使用无重力版`gesture_arm.urdf.xacro`，在各link中设置：

```xml
<gravity>false</gravity>
```

## 11.后续可扩展方向

1.接入MoveIt，实现末端位姿控制。
2.增加抓取目标，实现完整任务流程。
3.支持用户自定义手势。
4.统计用户操作时间、误触率和学习成本，形成HCI评价指标。
5.扩展到真实机械臂或已有的Gazebo机器狗项目。
