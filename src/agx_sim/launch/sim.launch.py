"""agx_sim：整套仿真的唯一入口。

参数（ros2 launch agx_sim sim.launch.py 参数:=值）：
    moveit        true    是否启动 MoveIt（开启后多出 /target_pose 话题）
    gui           true    是否启动 Gazebo 界面
    rviz          true    是否启动 RViz（moveit 开关决定用哪套 rviz 配置）
    aruco_dict    DICT_6X6_250  ArUco 字典名（cv2.aruco 里的名字）
    aruco_id      0       ArUco id
    aruco_size    0.16    ArUco 边长（m）
    aruco_vertical true   码竖放（true，面朝机械臂）/ 平放（false，躺在台面上）
    aruco_dx/dy   0.0     码相对凸台中心的偏移（m）
    table_x/y/z   0.45/0.0/0.15  凸台中心坐标（m，world 系 = base_link 系）
    table_sx/sy/sz 0.6/0.6/0.3   凸台尺寸（m）
    blocks        4      台面上放几个可以被夹爪抓起来的小方块（0=不放）

对外接口：
    /joint_states                        关节状态（仿真实时反馈）
    /arm_tcp_state                       夹爪 TCP 位姿（PoseStamped，基坐标系）
    /arm_controller/joint_trajectory     关节轨迹（JointTrajectory）-> 臂
    /gripper_controller/joint_trajectory 关节轨迹 -> 夹爪
    /target_pose                         目标位姿（PoseStamped）-> MoveIt 规划并执行（moveit:=true）
    /target_pose_status                  上面这条的执行结果（std_msgs/String）
"""

from pathlib import Path

import cv2
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ---------------------------------------------------------------- ArUco 几何
def _aruco_grid(dict_name, marker_id, cell_px=20):
    """把 ArUco 码转成 0/1 网格（1=黑格），含 1 格黑边。"""
    dict_id = getattr(cv2.aruco, dict_name)
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
    else:
        dictionary = cv2.aruco.Dictionary_get(dict_id)
    side = (dictionary.markerSize + 2) * cell_px
    if hasattr(cv2.aruco, "generateImageMarker"):
        img = cv2.aruco.generateImageMarker(dictionary, marker_id, side)
    else:
        img = cv2.aruco.drawMarker(dictionary, marker_id, side, 1)
    n = img.shape[0] // cell_px
    grid = [[int(img[r * cell_px + cell_px // 2, c * cell_px + cell_px // 2] < 128)
             for c in range(n)] for r in range(n)]
    return grid


def _aruco_sdf(name, dict_name, marker_id, size, pose):
    """用白底板 + 黑格子拼出码（贴图都不用），pose='x y z r p y'。"""
    grid = _aruco_grid(dict_name, marker_id)
    n = len(grid)
    cell = size / n
    half = size / 2.0
    plate = size + 0.06          # 白底板，四周留 quiet zone
    plate_t, cell_t = 0.004, 0.001
    out = [
        f'<model name="{name}"><static>true</static><pose>{pose}</pose>'
        '<link name="marker_link">',
        # 白底板
        '<visual name="plate"><cast_shadows>false</cast_shadows><geometry><box>'
        f'<size>{plate:.4f} {plate:.4f} {plate_t}</size></box></geometry>'
        '<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual>',
    ]
    # 黑格子
    for r in range(n):
        for c in range(n):
            if not grid[r][c]:
                continue
            x = -half + cell * (c + 0.5)
            y = half - cell * (r + 0.5)
            z = plate_t / 2.0 + cell_t / 2.0
            out.append(
                f'<visual name="c{r}{c}"><cast_shadows>false</cast_shadows>'
                f'<pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose><geometry><box>'
                f'<size>{cell:.4f} {cell:.4f} {cell_t}</size></box></geometry>'
                '<material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse>'
                '</material></visual>'
            )
    out.append('</link></model>')
    return "".join(out)


def _flag(value):
    """launch 参数都是字符串，这里统一成 bool。"""
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _blocks_sdf(tx, ty, top, hx, hy, count):
    """台面上放几个可抓取的小方块（非固定，带摩擦和接触刚度）。"""
    if count <= 0:
        return ""
    # (相对台面中心的偏移比例, 边长, 质量, 颜色)
    layout = [
        (-0.35, -0.35, 0.035, 0.05, "0.85 0.25 0.2 1"),
        (-0.35, 0.35, 0.030, 0.04, "0.2 0.55 0.85 1"),
        (0.05, -0.62, 0.040, 0.06, "0.95 0.75 0.15 1"),
        (0.05, 0.62, 0.025, 0.03, "0.3 0.8 0.35 1"),
        (-0.70, 0.0, 0.045, 0.08, "0.6 0.4 0.85 1"),
        (-0.05, 0.0, 0.030, 0.04, "0.9 0.5 0.2 1"),
    ]
    out = []
    for i in range(min(count, len(layout))):
        fx, fy, a, m, color = layout[i]
        x = tx + fx * hx
        y = ty + fy * hy
        z = top + a / 2.0 + 0.001
        inertia = m * a * a / 6.0
        out.append(f'''
    <model name="block_{i}">
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>{m}</mass>
          <inertia ixx="{inertia:.6g}" ixy="0" ixz="0"
                   iyy="{inertia:.6g}" iyz="0" izz="{inertia:.6g}"/>
        </inertial>
        <collision name="collision">
          <geometry><box><size>{a} {a} {a}</size></box></geometry>
          <surface>
            <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
            <contact><ode><kp>1e6</kp><kd>10</kd>
              <max_vel>0.01</max_vel><min_depth>0.001</min_depth></ode></contact>
          </surface>
        </collision>
        <visual name="visual">
          <geometry><box><size>{a} {a} {a}</size></box></geometry>
          <material>
            <ambient>{color}</ambient>
            <diffuse>{color}</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>''')
    return "".join(out)


# ------------------------------------------------------------------ world
def _world_sdf(ctx):
    """按参数生成 world：地面 + 凸台 + ArUco 码。"""
    tx = float(ctx["table_x"])
    ty = float(ctx["table_y"])
    tz = float(ctx["table_z"])
    sx = float(ctx["table_sx"])
    sy = float(ctx["table_sy"])
    sz = float(ctx["table_sz"])
    size = float(ctx["aruco_size"])
    dx = float(ctx["aruco_dx"])
    dy = float(ctx["aruco_dy"])
    top = tz + sz / 2.0                      # 台面高度
    plate_t = 0.004
    extra = ""

    if _flag(ctx["aruco_vertical"]):          # 竖放：面朝 -X（朝着机械臂）
        pose = (f"{tx + dx:.4f} {ty + dy:.4f} {top + (size + 0.06) / 2.0:.4f} "
                "0 -1.5707963 0")
        # 后面加个小支架，看起来像插在台上
        sx_ = tx + dx + (size + 0.06) / 2.0 + 0.015
        extra = (
            '<model name="aruco_stand"><static>true</static>'
            f'<pose>{sx_:.4f} {ty + dy:.4f} {top + 0.05:.4f} 0 0 0</pose>'
            '<link name="l"><collision name="c"><geometry><box>'
            '<size>0.03 0.06 0.10</size></box></geometry></collision>'
            '<visual name="v"><geometry><box><size>0.03 0.06 0.10</size></box>'
            '</geometry><material><ambient>0.3 0.3 0.33 1</ambient>'
            '<diffuse>0.45 0.45 0.5 1</diffuse></material></visual></link></model>'
        )
    else:                                    # 平放：躺在台面上
        pose = f"{tx + dx:.4f} {ty + dy:.4f} {top + plate_t / 2.0:.4f} 0 0 0"

    marker = _aruco_sdf("aruco_marker", ctx["aruco_dict"], int(ctx["aruco_id"]),
                        size, pose)
    # 台面上可抓取的小方块（留出边距，保证落在台面上）
    hx = max(sx / 2.0 - 0.05, 0.02)
    hy = max(sy / 2.0 - 0.05, 0.02)
    blocks = _blocks_sdf(tx, ty, top, hx, hy, int(ctx["blocks"]))

    return f'''<?xml version="1.0"?>
<sdf version="1.6">
  <world name="agx_sim">
    <physics name="ode" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.81</gravity>

    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 5 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.3 -0.9</direction>
    </light>

    <!-- 地面 -->
    <model name="ground"><static>true</static><link name="l">
      <collision name="c"><geometry><plane><normal>0 0 1</normal>
        <size>20 20</size></plane></geometry></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal>
        <size>20 20</size></plane></geometry>
        <material><ambient>0.75 0.75 0.75 1</ambient>
        <diffuse>0.8 0.8 0.8 1</diffuse></material></visual>
    </link></model>

    <!-- 凸台（参数化） -->
    <model name="table"><static>true</static>
      <pose>{tx:.4f} {ty:.4f} {tz:.4f} 0 0 0</pose><link name="l">
      <collision name="c"><geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size>
        </box></geometry></collision>
      <visual name="v"><geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box>
        </geometry><material><ambient>0.45 0.35 0.25 1</ambient>
        <diffuse>0.6 0.47 0.32 1</diffuse></material></visual>
    </link></model>

    {marker}
    {extra}
    {blocks}

    <scene><ambient>0.5 0.5 0.5 1</ambient><background>0.7 0.7 0.7 1</background>
    </scene>
  </world>
</sdf>
'''


# ------------------------------------------------------------------ 启动
def _launch(context, *args, **kwargs):
    ctx = {k: v.perform(context) for k, v in ARGS.items()}
    share = Path(get_package_share_directory("agx_sim"))
    model_share = Path(get_package_share_directory("agx_arm_description")).parent

    # 生成的 world 放 /tmp，避免污染包目录
    world_path = Path("/tmp/agx_sim/sim.world")
    world_path.parent.mkdir(parents=True, exist_ok=True)
    world_path.write_text(_world_sdf(ctx))

    robot_description = {
        "robot_description": xacro.process_file(
            str(share / "config" / "agx_arm.urdf.xacro")
        ).toxml(),
        "use_sim_time": True,
    }

    actions = [
        # 别去连 gazebosim 模型库，否则没网时 GUI 会卡住
        SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),
        # package:// -> model:// 需要这个
        AppendEnvironmentVariable("GAZEBO_MODEL_PATH", str(model_share)),
    ]
    # libgazebo_ros_camera.so 依赖 gazebo 自带的 libCameraPlugin.so，它不在默认 linker 路径里
    actions.append(AppendEnvironmentVariable(
        "LD_LIBRARY_PATH", "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins"))

    spawn_robot = Node(
        package="gazebo_ros", executable="spawn_entity.py", output="log",
        arguments=["-entity", "agx_arm", "-topic", "robot_description"])

    actions += [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(get_package_share_directory("gazebo_ros"))
                    / "launch" / "gzserver.launch.py")
            ),
            launch_arguments={"world": str(world_path)}.items(),
        ),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             output="log", parameters=[robot_description]),
        spawn_robot,
    ]

    # Gazebo 界面（用裸 gzclient：gazebo_ros 自带的 EOL 插件在 11.10 会崩）
    actions.append(ExecuteProcess(
        cmd=["gzclient"], output="log", condition=IfCondition(LaunchConfiguration("gui"))))

    # 机器人生成后再拉起控制器
    spawners = [
        Node(package="controller_manager", executable="spawner", output="log",
             arguments=["joint_state_broadcaster", "--controller-manager",
                        "/controller_manager", "--controller-manager-timeout", "60"]),
        Node(package="controller_manager", executable="spawner", output="log",
             arguments=["arm_controller", "--controller-manager",
                        "/controller_manager", "--controller-manager-timeout", "60"]),
        Node(package="controller_manager", executable="spawner", output="log",
             arguments=["gripper_controller", "--controller-manager",
                        "/controller_manager", "--controller-manager-timeout", "60"]),
    ]
    actions.append(RegisterEventHandler(OnProcessExit(
        target_action=spawn_robot, on_exit=spawners)))
    last_spawner = spawners[-1]

    # 夹爪 TCP 位姿反馈（纯反馈，与 moveit 无关，始终启动）
    actions.append(Node(package="agx_sim", executable="tcp_state_publisher",
                        output="log", parameters=[{"use_sim_time": True}]))

    # MoveIt：move_group + /target_pose 桥接节点（moveit:=false 时完全不加载）
    moveit_params = None
    if _flag(ctx["moveit"]):
        try:
            from moveit_configs_utils import MoveItConfigsBuilder
        except ImportError as exc:
            raise RuntimeError(
                "moveit:=true 需要 MoveIt，但找不到 moveit_configs_utils。\n"
                "  安装：sudo apt install ros-humble-moveit\n"
                "  或关掉 MoveIt：ros2 launch agx_sim sim.launch.py moveit:=false\n"
                f"  （原始错误：{exc}）"
            ) from exc

        cfg = (MoveItConfigsBuilder("agx_arm", package_name="agx_sim")
               .robot_description(file_path="config/agx_arm.urdf.xacro")
               .trajectory_execution(file_path="config/moveit_controllers.yaml")
               .to_moveit_configs())
        moveit_params = cfg.to_dict()
        actions += [
            Node(package="moveit_ros_move_group", executable="move_group",
                 output="log", parameters=[moveit_params, {"use_sim_time": True}]),
            Node(package="agx_sim", executable="target_pose_bridge", output="log",
                 parameters=[{"use_sim_time": True, "planning_group": "arm",
                              "ee_link": "link6"}]),
        ]

    # RViz：开了 MoveIt 用带 MotionPlanning 面板的那套配置，否则用基础配置
    if _flag(ctx["rviz"]):
        rviz_config = "rviz_moveit.rviz" if moveit_params else "rviz_sim.rviz"
        rviz_params = dict(moveit_params) if moveit_params else {}
        rviz_params["use_sim_time"] = True
        # 等控制器拉起后再起 RViz：RViz 启动时若还没有相机话题，图像显示不会订阅
        rviz_node = Node(package="rviz2", executable="rviz2", output="log",
                         arguments=["-d", str(share / "config" / rviz_config)],
                         parameters=[rviz_params])
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=last_spawner,
            on_exit=[TimerAction(period=20.0, actions=[rviz_node])])))
    return actions


ARGS = {
    "moveit": LaunchConfiguration("moveit"),
    "gui": LaunchConfiguration("gui"),
    "rviz": LaunchConfiguration("rviz"),
    "aruco_dict": LaunchConfiguration("aruco_dict"),
    "aruco_id": LaunchConfiguration("aruco_id"),
    "aruco_size": LaunchConfiguration("aruco_size"),
    "aruco_vertical": LaunchConfiguration("aruco_vertical"),
    "aruco_dx": LaunchConfiguration("aruco_dx"),
    "aruco_dy": LaunchConfiguration("aruco_dy"),
    "table_x": LaunchConfiguration("table_x"),
    "table_y": LaunchConfiguration("table_y"),
    "table_z": LaunchConfiguration("table_z"),
    "table_sx": LaunchConfiguration("table_sx"),
    "table_sy": LaunchConfiguration("table_sy"),
    "table_sz": LaunchConfiguration("table_sz"),
    "blocks": LaunchConfiguration("blocks"),
}


def generate_launch_description():
    declared = [
        DeclareLaunchArgument("moveit", default_value="true",
                              description="是否启动 MoveIt（/target_pose 话题）"),
        DeclareLaunchArgument("gui", default_value="true",
                              description="是否启动 Gazebo 界面"),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="是否启动 RViz"),
        DeclareLaunchArgument("aruco_dict", default_value="DICT_6X6_250",
                              description="ArUco 字典名"),
        DeclareLaunchArgument("aruco_id", default_value="0", description="ArUco id"),
        DeclareLaunchArgument("aruco_size", default_value="0.16",
                              description="ArUco 边长（m）"),
        DeclareLaunchArgument("aruco_vertical", default_value="true",
                              description="true=竖放，false=平放"),
        DeclareLaunchArgument("aruco_dx", default_value="0.0",
                              description="码相对凸台中心的 x 偏移（m）"),
        DeclareLaunchArgument("aruco_dy", default_value="0.0",
                              description="码相对凸台中心的 y 偏移（m）"),
        DeclareLaunchArgument("table_x", default_value="0.45",
                              description="凸台中心 x（m）"),
        DeclareLaunchArgument("table_y", default_value="0.0",
                              description="凸台中心 y（m）"),
        DeclareLaunchArgument("table_z", default_value="0.15",
                              description="凸台中心 z（m）"),
        DeclareLaunchArgument("table_sx", default_value="0.6",
                              description="凸台长（m）"),
        DeclareLaunchArgument("table_sy", default_value="0.6",
                              description="凸台宽（m）"),
        DeclareLaunchArgument("table_sz", default_value="0.3",
                              description="凸台高（m）"),
        DeclareLaunchArgument("blocks", default_value="4",
                              description="台面上小方块的个数（0=不放）"),
    ]
    return LaunchDescription(declared + [OpaqueFunction(function=_launch)])
