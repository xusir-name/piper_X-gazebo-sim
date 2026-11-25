# arm_sim — piper_x 最简 Gazebo 仿真

一个只有两个包的 colcon 工作区：模型 + 仿真。**纯接口，不含任何业务逻辑**，用于给外部算法提供一个
可控制、可观测的机械臂仿真环境。

```
arm_sim/
├── src/agx_arm_description/   # 精简模型：仅 piper_x（URDF + mesh）
└── src/agx_sim/               # 仿真：1 个 launch + 配置文件 + 1 个 python 节点
    ├── launch/sim.launch.py   # 唯一入口（按参数生成 world）
    ├── scripts/target_pose_bridge.py
    └── config/                # urdf/xacro、srdf、controller、MoveIt、rviz 配置
```

## 1. 依赖

ROS 2 Humble（Ubuntu 22.04）+ Gazebo Classic 11。

```bash
# 仿真、控制器、相机插件
sudo apt install -y \
  ros-humble-gazebo-ros ros-humble-gazebo-plugins ros-humble-gazebo-ros2-control \
  ros-humble-controller-manager ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller ros-humble-robot-state-publisher \
  ros-humble-xacro ros-humble-cv-bridge python3-opencv \
  ros-humble-rviz2 ros-humble-rviz-default-plugins

# 可选：只有 moveit:=true 时才需要（提供 moveit_configs_utils 等）
# 没装的话用 moveit:=false 启动，否则 launch 会提示缺 moveit_configs_utils
sudo apt install -y ros-humble-moveit
```

> 相机话题（`/camera/...`）由 `ros-humble-gazebo-plugins` 里的 `libgazebo_ros_camera.so`
> 在 Gazebo 里直接渲染发布，**不需要另开终端跑 RealSense 驱动**（也没接真实相机）。
> 缺这个包时 launch 会打印警告，`/camera/*` 话题不会出现。

或者用 rosdep 一次装完：

```bash
cd arm_sim
rosdep install --from-paths src --ignore-src -y
```

## 2. 编译与运行

```bash
cd arm_sim
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

ros2 launch agx_sim sim.launch.py            # 默认：Gazebo 界面 + MoveIt
ros2 launch agx_sim sim.launch.py moveit:=false   # 只要关节接口（RViz 自动换成基础配置）
ros2 launch agx_sim sim.launch.py rviz:=false     # 不要 RViz
ros2 launch agx_sim sim.launch.py gui:=false      # 不要 Gazebo 界面（无显示器/远程）
```

## 3. launch 参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `moveit` | `true` | 是否启动 MoveIt（`false` 时不加载 `moveit_configs_utils`，没装 MoveIt 也能跑） |
| `gui` | `true` | 是否启动 Gazebo 界面（gzclient） |
| `rviz` | `true` | 是否启动 RViz；`moveit` 决定用哪套配置（见下） |
| `aruco_dict` | `DICT_6X6_250` | ArUco 字典名（`cv2.aruco` 里的名字，如 `DICT_4X4_50`） |
| `aruco_id` | `0` | 码 id |
| `aruco_size` | `0.16` | 码边长（m） |
| `aruco_vertical` | `true` | `true` 竖放（码面朝机械臂）/ `false` 平放（躺在台面上） |
| `aruco_dx` / `aruco_dy` | `0.0` | 码相对凸台中心的偏移（m） |
| `table_x` / `table_y` / `table_z` | `0.45` / `0.0` / `0.15` | 凸台中心坐标（m，原点在机械臂底座） |
| `table_sx` / `table_sy` / `table_sz` | `0.6` / `0.6` / `0.3` | 凸台尺寸（m），台面高度 = `table_z + table_sz/2` |
| `blocks` | `4` | 台面上放几个可抓取的小方块（边长 2.5~4.5 cm，非固定、带摩擦），`0`=不放 |
| `kill_stale` | `true` | 启动前清掉上次没退干净的 `gzserver`/`gzclient`（只认本仿真的 world） |

```bash
# 4x4 码、平放、换个凸台
ros2 launch agx_sim sim.launch.py aruco_dict:=DICT_4X4_50 aruco_id:=3 aruco_size:=0.10 \
  aruco_vertical:=false table_x:=0.40 table_z:=0.12 table_sx:=0.5 table_sy:=0.4 table_sz:=0.24
```

## 4. 启动流程

`sim.launch.py` 是一个 `OpaqueFunction`，按参数生成 world 后依次拉起：

```
1. gzserver（gazebo_ros/gzserver.launch.py）
      world = /tmp/agx_sim/sim.world（按参数生成：地面 + 凸台 + ArUco 码）
2. gzclient                                        # gui:=true
3. robot_state_publisher                           # 解析 config/agx_arm.urdf.xacro
4. spawn_entity.py -topic robot_description         # 把机器人放进 Gazebo
      └─ 进程退出后（OnProcessExit）拉起：
5. controller_manager/spawner x3
      joint_state_broadcaster -> arm_controller -> gripper_controller
      （控制器由 gazebo_ros2_control 加载 config/ros2_controllers.yaml）
6. tcp_state_publisher（agx_sim/tcp_state_publisher）
      —— 查 TF 算出夹爪两指指尖中点，发给 /arm_tcp_state（50 Hz）
7. move_group（moveit_ros_move_group）              # moveit:=true
      —— 轨迹经 moveit_simple_controller_manager 发给上面两个
         FollowJointTrajectory 控制器
8. target_pose_bridge（agx_sim/target_pose_bridge） # moveit:=true
      —— /target_pose -> /move_action -> 控制器
9. rviz2                                        # rviz:=true
      —— 等控制器拉起后再延迟 20 s 启动，确保相机话题已存在，否则图像显示不会订阅
```

RViz 两套配置，按 `moveit` 自动选：

| 文件 | 何时使用 | 内容 |
| --- | --- | --- |
| `config/rviz_sim.rviz` | `moveit:=false` | Grid、RobotModel、TF、彩色图、深度图 |
| `config/rviz_moveit.rviz` | `moveit:=true` | 上面这些 + `MotionPlanning` 面板（规划/执行/场景） |

机器人本体（`config/agx_arm.urdf.xacro`）= piper_x 臂 + 电动夹爪 + 夹爪上的 RealSense D435
（`config/d435_mount.urdf.xacro`，安装尺寸照抄 AgileX 官方
`piper_x_gripper_d435.urdf`），相机传感器由 `libgazebo_ros_camera.so`（`gazebo_plugins`）提供。

## 5. 对外接口

| 话题 | 消息类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/joint_states` | `sensor_msgs/msg/JointState` | 输出 | 关节位置/速度（仿真反馈） |
| `/arm_tcp_state` | `geometry_msgs/msg/PoseStamped` | 输出 | 夹爪 TCP（两指指尖中点）在 `base_link` 下的位姿 |
| `/arm_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | 输入 | 直接给 6 个臂关节发轨迹 |
| `/gripper_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | 输入 | 夹爪开合（`gripper_joint1/2`，±0.05 rad） |
| `/target_pose` | `geometry_msgs/msg/PoseStamped` | 输入 | 末端目标位姿，MoveIt 后台规划并执行（`moveit:=true`） |
| `/target_pose_status` | `std_msgs/msg/String` | 输出 | `accepted` / `planning` / `succeeded` / `failed: ...` / `busy` |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | 输出 | 夹爪 D435 彩色图（rgb8，640x480，15 Hz） |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 输出 | 相机内参 |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | 输出 | 深度图（32FC1） |
| `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 输出 | 深度相机内参 |
| `/camera/points` | `sensor_msgs/msg/PointCloud2` | 输出 | 点云 |
| `/clock` | `rosgraph_msgs/msg/Clock` | 输出 | 仿真时钟（外部节点建议 `use_sim_time:=true`） |

TF 由 `robot_state_publisher` 发布：`base_link`（世界原点，位于地面）→ `link1..link6` →
`gripper_base` → `camera_link` → `camera_optical_frame`（REP-103 光学系）。

### 用法示例

```bash
# 关节空间控制：3 秒走到目标角
ros2 topic pub --once /arm_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [joint1, joint2, joint3, joint4, joint5, joint6],
    points: [{positions: [0.5, 0.8, -0.9, 0.2, 0.3, 0.1],
              time_from_start: {sec: 3, nanosec: 0}}]}"

# 夹爪闭合
ros2 topic pub --once /gripper_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [gripper_joint1, gripper_joint2],
    points: [{positions: [0.0, 0.0], time_from_start: {sec: 2, nanosec: 0}}]}"

# 末端位姿目标（orientation 全 0 = 姿态自由）
ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base_link},
    pose: {position: {x: 0.30, y: 0.0, z: 0.35},
           orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}}"
ros2 topic echo /target_pose_status
```

## 6. 说明

- 台面上的小方块是普通动态刚体（`static=false`，质量 0.03~0.06 kg，`mu=1.0`，
  接触刚度 `kp=1e6`），夹爪闭合就能夹住拿走；它们的位置按凸台尺寸等比计算，跟着 `table_sx/sy` 走。
- `/arm_tcp_state` 的 TCP 取在 `gripper_base` 沿 +Z 0.1358 m 处，正好是两个指尖 frame 的中点，
  姿态沿用 `gripper_base`；换夹爪或想改抓取点时用节点参数 `tcp_z` / `tcp_parent` 调。
- 如果上次的 Gazebo 没退干净（比如 `kill -9` 或直接关终端），11345 端口会被占着，
  新 `gzserver` 会静默退出、`spawn_entity` 报 "Entity already exists"，于是你看到的还是旧世界、
  关节角也不会重置。`kill_stale:=true`（默认）会在启动前自动清理本仿真残留的进程；
  设成 `false` 时若端口仍被占用会直接报错提示。
- 初始关节角：`joint2=1.2, joint3=-1.2`，其余 0，夹爪 `±0.02`。
- ArUco 码由白底板 + 黑格子拼成（不用贴图），每次 launch 按参数写入 `/tmp/agx_sim/sim.world`；
  白底板比码大 6 cm 以留出 quiet zone，且关闭投影避免自阴影影响检测。
- 为了让包足够小：模型包只保留 piper_x；相机本体用方块代替 16 MB 的 `d435.dae`，并**不加碰撞体**
  （不参与 MoveIt 碰撞检测）。
- 若外部节点使用仿真时间，记得设置 `use_sim_time:=true`。
- 没有 `/camera/*` 话题 = 缺 `ros-humble-gazebo-plugins`（不是需要 RealSense 驱动），
  装上重启即可；launch 启动时会给出对应警告。
