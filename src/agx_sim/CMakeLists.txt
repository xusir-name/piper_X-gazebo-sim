cmake_minimum_required(VERSION 3.16)
project(agx_sim)

find_package(ament_cmake REQUIRED)

# launch / config
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# 目标位姿 -> MoveIt 桥接节点
install(PROGRAMS scripts/target_pose_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME target_pose_bridge)

# 夹爪 TCP 位姿反馈节点
install(PROGRAMS scripts/tcp_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME tcp_state_publisher)

ament_package()
