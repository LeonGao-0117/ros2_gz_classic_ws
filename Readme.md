# M3Pro Robot Simulation Run & Installation Tutorial (M3Pro 机器人仿真运行与安装教程)

[English](#english-version) | [中文版](#中文版)

---

<a name="english-version"></a>

## English Version

This tutorial will guide you on how to setup your environment, install dependencies, and start the M3Pro robot's simulation and visualization interface.

### 1. Environment Dependencies Installation (Ubuntu 22.04 + ROS 2 Humble)

Before running the simulation, please assure that your system has installed ROS 2 Humble, and basic Gazebo environment is configured.
The following are the required dependencies needed to run the M3Pro robot. Please execute the following commands in your terminal to install them:

```bash
# Update software sources
sudo apt update

# 1. Install controller_manager core package (used to load and manage control plugins)
sudo apt install ros-humble-controller-manager

# 2. Install common controllers and control interfaces (essential for robotic arm and chassis)
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers


# 3. Install Gazebo Fortress exclusive plugin for controllers (Core!)
sudo apt install ros-humble-ign-ros2-control

# Install Gazebo Fortress integration tools with ROS (including Bridge)
sudo apt install ros-humble-ros-gz

# 4. Install Joint State Publisher GUI (used to control joints via sliders in RViz)
sudo apt install ros-humble-joint-state-publisher-gui

# 5. Install topic_tools (used for topic relaying, such as relaying /cmd_vel)
sudo apt install ros-humble-topic-tools

# 6. Install keyboard control node (used to drive chassis via keyboard inputs)
sudo apt install ros-humble-teleop-twist-keyboard
```

*Note: The `$ROS_DISTRO` in the above commands is explicitly replaced with `humble`, since M3Pro uses the ROS 2 Humble version of Ubuntu 22.04.*

---

### 2. Compile and Configure Workspace

Note that this workspace mainly contains four packages: `M3Pro_robot_description`, `M3Pro_robot_bringup`, `aws_robomaker_hospital_world`, and `aws_robomaker_small_house_world`.

```bash
# 1. Enter the root directory of the workspace
cd ~/ros2_ws  # Please replace with your actual workspace path

# 2. Compile the workspace
colcon build

# 3. Source environment variables to make packages effective
source install/setup.bash
```

*(Tip: It is recommended to add `source ~/ros2_ws/install/setup.bash` to your `~/.bashrc` file, so that it loads automatically every time you open a terminal.)*

---

### 3. Run Tutorial

#### Scenario 1: Verify robot model only in RViz (URDF display)

This function is solely utilized to browse and confirm the robot's URDF model, without physical simulation. Under this mode, it pops up a GUI window equipped with sliders. You can configure and review every joint's movement by dragging the sliders.

**Run Command:**
```bash
ros2 launch M3Pro_robot_description display.launch.py
```

**You will see:**
1. A popped up RViz2 window displaying the M3Pro robot model.
2. A popped up `joint_state_publisher_gui` window, where you can drag sliders to change the pose of the robotic arm and wheels directly.

---

#### Scenario 2: Start Complete Gazebo Simulation System

This function will start a physical simulation environment powered by Gazebo, loading the control plugins (Mecanum wheel chassis and robotic arm), and triggering simulations for sensors like depth camera, LiDAR, etc.

**Run Command (default world):**
```bash
ros2 launch M3Pro_robot_bringup M3Pro_robot.launch.py
```

**Run Command (AWS hospital world):**
```bash
ros2 launch M3Pro_robot_bringup M3Pro_robot.launch.py world:=hospital
```

**Run Command (AWS small house world):**
```bash
ros2 launch M3Pro_robot_bringup M3Pro_robot.launch.py world:=small_house
```

**The system will complete the startup automatically following these steps:**
1. Start Gazebo Sim and load the `M3pro_world.sdf` simulation world.
2. Spawn the M3Pro robot model into Gazebo.
3. Start `ros_gz_bridge` to establish topic bridges between Gazebo and ROS 2 (e.g. sensor data `/scan`, `/camera/image_raw`, clock `/clock`, etc.).
4. Delayed loading of the `controller_manager` as well as various associated controller plugins (joint state publisher, arm controller, mecanum chassis controller, etc.).
5. Automatically open and configure the RViz2 UI for real-time status monitoring.
6. Start `cmd_vel_relay`, allowing users to send commands directly to the standard `/cmd_vel` topic to control the chassis.

---

### 4. Control and Test Simulation Environment

Once **Scenario 2** initiates successfully, you can open a new terminal for the tests stated below (before opening any new terminals, assure you've executed `source install/setup.bash` on them):

#### 4.1 Chassis Control

##### 4.1.1 Command Line: Publish Twist to `/cmd_vel`

The chassis is controlled by **libgazebo_ros_planar_move**, and subscribes to `geometry_msgs/msg/Twist`.

**Move forward (linear velocity x=0.2 m/s):**
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**Turn left (angular velocity z=0.5 rad/s):**
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

---

### 5. Robotic Arm Control (Joint Angles)

The robotic arm is controlled by **arm_controller** (JointTrajectoryController), the interface is **position**, unit is **radians (rad)**.

Joint order and names:
- `arm1_Joint` ~ `arm5_Joint`: 5 joints of the robotic arm

#### 5.1 Command Line: Send single point target

**Send a target configuration (5 joints, unit rad):**
```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{
    trajectory: {
      joint_names: [arm1_Joint, arm2_Joint, arm3_Joint, arm4_Joint, arm5_Joint],
      points: [
        {
          positions: [0.0, 0.0, 0.0, 0.0, 0.0],
          time_from_start: {sec: 2, nanosec: 0}
        }
      ]
    }
  }"
```

**Example: arm1 rotates to 0.5 rad, others to 0:**
```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{
    trajectory: {
      joint_names: [arm1_Joint, arm2_Joint, arm3_Joint, arm4_Joint, arm5_Joint],
      points: [
        {
          positions: [0.5, 0.0, 0.0, 0.0, 0.0],
          time_from_start: {sec: 2, nanosec: 0}
        }
      ]
    }
  }"
```

---

### 6. Gripper Control

The gripper is controlled separately by **gripper_controller** (GripperActionController).

- `rlink1_Joint`: Gripper joint

#### 6.1 Command Line: Open/Close Gripper

```bash
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand \
  "{
    command: {
      position: 0.0,
      max_effort: 5.0
    }
  }"
```
*(You can adjust the `position` value to open or close the gripper, unit often ranges around 0.0 to -0.7854 depending on URDF limits).*

**View current joint status:**
```bash
ros2 topic echo /joint_states
```

---

### 7. How to drive from the application layer

#### 7.1 Chassis: Publish `/cmd_vel` (Python)

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ChassisDriver(Node):
    def __init__(self):
        super().__init__('chassis_driver')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def move(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = float(linear_y)
        msg.angular.z = float(angular_z)
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = ChassisDriver()
    node.move(0.2, 0.0, 0.0)  # Move forward
    rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()
```

---

### 8. LiDAR and Depth Camera (How to enable and use)

LiDAR and depth camera have been configured as Gazebo plugins in **M3Pro_robot_sensors.xacro**, **as long as the robot is spawned into Gazebo using `M3Pro_robot.launch.py`, they will run automatically**, no extra enable commands are needed.

**(Optional) To visualize LiDAR rays in Gazebo:** You need to explicitly enable them in the Gazebo Sim GUI. Click the top-right menu (3 vertical dots) -> `Plugins` -> `Visualize Lidar`, or press the corresponding shortcut if configured.

#### 8.1 LiDAR

| Description | Front LiDAR     | Rear LiDAR      |
|-------------|-----------------|-----------------|
| Topic       | `/scan`         | `/scan_back`    |
| Message Type| `sensor_msgs/msg/LaserScan` | Same as left    |

**View front LiDAR:**
```bash
ros2 topic echo /scan
```

**View in RViz:**
- Add LaserScan display, Topic select `/scan` or `/scan_back`, Fixed Frame select `base_footprint` or `odom`.

---

### 9. Summary of Common Topics and Actions

| Function           | Type             | Topic / Action |
|--------------------|------------------|----------------|
| Chassis Control    | Topic (Twist)    | `/cmd_vel` |
| Arm Control        | Action           | `/arm_controller/follow_joint_trajectory` |
| Gripper Control    | Action           | `/gripper_controller/gripper_cmd` |
| Joint States       | Topic            | `/joint_states` |
| Odometry           | Topic            | `/odom` |
| Front LiDAR        | Topic (LaserScan) | `/scan` |
| Rear LiDAR         | Topic (LaserScan) | `/scan_back` |
| Camera Image       | Topic (Image)    | `/camera/image` |
| Camera Info        | Topic            | `/camera/camera_info` |

Robotic arm has 5 joints: `arm1_Joint` ~ `arm5_Joint`. Control interface is **position**, unit is **rad**.
Gripper has 1 joint: `rlink1_Joint`.

---

<a name="中文版"></a>

## 中文版

本教程将指导您如何配置环境、安装依赖，并启动 M3Pro 机器人的仿真与可视化界面。

### 1. 环境依赖安装 (Ubuntu 22.04 + ROS 2 Humble)

在运行仿真前，请确保系统已安装 ROS 2 Humble，并且配置好了基础的 Gazebo 环境。
以下是运行 M3Pro 机器人所需的依赖项。请在终端中执行以下命令进行安装：

```bash
# 更新软件源
sudo apt update

# 1. 安装 controller_manager 核心包 (用于加载和管理控制插件)
sudo apt install ros-humble-controller-manager

# 2. 安装常用的控制器和控制接口 (机械臂和底盘必用)
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers

# 3. 安装 Gazebo Fortress与控制器的专属插件 (核心！)
sudo apt install ros-humble-ign-ros2-control

# 安装 Gazebo Fortress与 ROS 的集成工具 (含 Bridge)
sudo apt install ros-humble-ros-gz

# 4. 安装 Joint State Publisher GUI (用于在 RViz 中通过滑块控制关节)
sudo apt install ros-humble-joint-state-publisher-gui

# 5. 安装 topic_tools (用于话题转发，如 /cmd_vel 的转发)
sudo apt install ros-humble-topic-tools

# 6. 安装键盘控制节点 (用于键盘按键驱动底盘)
sudo apt install ros-humble-teleop-twist-keyboard
```

*注意：以上命令中的 `$ROS_DISTRO` 已明确替换为 `humble`，因为 M3Pro 使用的是 Ubuntu 22.04 的 ROS 2 Humble 版本。*

---

### 2. 编译并配置工作空间

当前工作空间主要包含四个功能包：`M3Pro_robot_description`、`M3Pro_robot_bringup`、`aws_robomaker_hospital_world` 和 `aws_robomaker_small_house_world`。

```bash
# 1. 进入工作空间根目录
cd ~/ros2_ws  # 请替换为您的真实工作空间路径

# 2. 编译工作空间
colcon build

# 3. 环境变量 source，使包生效
source install/setup.bash
```

*(提示：建议将 `source ~/ros2_ws/install/setup.bash` 加入到您的 `~/.bashrc` 文件中，这样每次打开终端都会自动加载。)*

---

### 3. 运行教程

#### 场景一：只在 RViz 中查看机器人模型 (URDF 显示)

此功能仅用于查看和验证机器人的 URDF 模型，不包含物理仿真。在此模式下，系统会弹出一个带滑块的 GUI 窗口，您可以拖动滑块查看各个关节的运动情况。

**运行命令：**
```bash
ros2 launch M3Pro_robot_description display.launch.py
```

**您将看到：**
1. 弹出的 RViz2 窗口显示 M3Pro 机器人模型。
2. 弹出的 `joint_state_publisher_gui` 窗口，可直接拖动滑块改变机械臂和车轮的姿态。

---

#### 场景二：启动完整的 Gazebo 仿真系统

此功能将启动由 Gazebo 驱动的物理仿真环境，加载了控制插件（麦克纳姆轮底盘和机械臂），并且启动了深度相机、激光雷达等传感器仿真。

**运行命令（默认世界）：**
```bash
ros2 launch M3Pro_robot_bringup M3Pro_robot.launch.py
```

**运行命令（AWS 医院世界）：**
```bash
ros2 launch M3Pro_robot_bringup M3Pro_robot.launch.py world:=hospital
```

**运行命令（AWS 小户型房屋世界）：**
```bash
ros2 launch M3Pro_robot_bringup M3Pro_robot.launch.py world:=small_house
```

**系统将按照以下流程自动完成启动：**
1. 启动 Gazebo Sim 并加载 `M3pro_world.sdf` 仿真世界。
2. 将 M3Pro 机器人模型生成 (Spawn) 到 Gazebo 中。
3. 启动 `ros_gz_bridge` 建立 Gazebo 与 ROS 2 之间的话题桥接（如传感器数据 `/scan`, `/camera/image_raw`，时钟 `/clock` 等）。
4. 延迟加载 `controller_manager` 及相关的各种控制器插件（关节状态发布器、机械臂控制器、麦轮底盘控制器等）。
5. 自动启动并配置好 RViz2 界面用于实时状态监控。
6. 启动 `cmd_vel_relay`，使得用户可以直接往标准 `/cmd_vel` 话题发送指令控制底盘。

---

### 4. 控制与测试仿真环境

当 **场景二** 成功启动后，您可以打开新的终端进行如下测试（每次打开新终端前，请确保执行了 `source install/setup.bash`）：

#### 4.1 底盘控制

##### 4.1.1 命令行：发布 Twist 到 `/cmd_vel`

底盘由 **libgazebo_ros_planar_move** 控制，订阅 `geometry_msgs/msg/Twist`。

**前进（线速度 x=0.2 m/s）：**
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**左转（角速度 z=0.5 rad/s）：**
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

---

### 5. 机械臂控制 (关节角度)

机械臂由 **arm_controller** (JointTrajectoryController) 控制，接口为 **position**，单位为 **弧度 (rad)**。

关节顺序与名称：
- `arm1_Joint` ~ `arm5_Joint`：机械臂的 5 个关节

#### 5.1 命令行：发送单点目标

**发送目标位形 (5 个关节，单位 rad)：**
```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{
    trajectory: {
      joint_names: [arm1_Joint, arm2_Joint, arm3_Joint, arm4_Joint, arm5_Joint],
      points: [
        {
          positions: [0.0, 0.0, 0.0, 0.0, 0.0],
          time_from_start: {sec: 2, nanosec: 0}
        }
      ]
    }
  }"
```

**示例：arm1 转动到 0.5 rad，其余为 0：**
```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{
    trajectory: {
      joint_names: [arm1_Joint, arm2_Joint, arm3_Joint, arm4_Joint, arm5_Joint],
      points: [
        {
          positions: [0.5, 0.0, 0.0, 0.0, 0.0],
          time_from_start: {sec: 2, nanosec: 0}
        }
      ]
    }
  }"
```

---

### 6. 夹爪控制

夹爪由 **gripper_controller** (GripperActionController) 独立控制。

- `rlink1_Joint`：夹爪关节

#### 6.1 命令行：打开/关闭夹爪

```bash
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand \
  "{
    command: {
      position: 0.0,
      max_effort: 5.0
    }
  }"
```
*(您可以调整 `position` 值来打开或关闭夹爪，单位通常在 0.0 到 -0.7854 之间，取决于 URDF 限制)。*

**查看当前关节状态：**
```bash
ros2 topic echo /joint_states
```

---

### 7. 如何从应用层控制

#### 7.1 底盘：发布 `/cmd_vel` (Python)

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ChassisDriver(Node):
    def __init__(self):
        super().__init__('chassis_driver')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def move(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = float(linear_y)
        msg.angular.z = float(angular_z)
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = ChassisDriver()
    node.move(0.2, 0.0, 0.0)  # 前进
    rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()
```

---

### 8. 雷达和深度相机 (如何开启与使用)

雷达和深度相机已在 **M3Pro_robot_sensors.xacro** 中配置为 Gazebo 插件，**只要使用 `M3Pro_robot.launch.py` 将机器人生成到 Gazebo 中，它们就会自动运行**，无需额外的开启命令。

**(可选) 在 Gazebo 中可视化雷达射线：** 您需要在 Gazebo Sim GUI 中显式开启。点击右上角菜单 (3个点) -> `Plugins` -> `Visualize Lidar`。

#### 8.1 雷达

| 说明 | 前雷达 | 后雷达 |
|-------------|-----------------|-----------------|
| 话题 | `/scan` | `/scan_back` |
| 消息类型 | `sensor_msgs/msg/LaserScan` | 同上 |

**查看前雷达数据：**
```bash
ros2 topic echo /scan
```

**在 RViz 中查看：**
- 添加 LaserScan 显示，话题选择 `/scan` 或 `/scan_back`，Fixed Frame 选择 `base_footprint` or `odom`。

---

### 9. 常用话题与动作汇总

| 功能 | 类型 | 话题 / 动作 |
|--------------------|------------------|----------------|
| 底盘控制 | 话题 (Twist) | `/cmd_vel` |
| 机械臂控制 | 动作 | `/arm_controller/follow_joint_trajectory` |
| 夹爪控制 | 动作 | `/gripper_controller/gripper_cmd` |
| 关节状态 | 话题 | `/joint_states` |
| 里程计 | 话题 | `/odom` |
| 前雷达 | 话题 (LaserScan) | `/scan` |
| 后雷达 | 话题 (LaserScan) | `/scan_back` |
| 相机图像 | 话题 (Image) | `/camera/image` |
| 相机内参 | 话题 | `/camera/camera_info` |

机械臂拥有 5 个关节：`arm1_Joint` ~ `arm5_Joint`。控制接口为 **position**，单位为 **rad**。
夹爪拥有 1 个关节：`rlink1_Joint`。
