#!/usr/bin/env python3
"""目标位姿 -> MoveIt 规划 + 执行。

订阅: /target_pose        geometry_msgs/PoseStamped（frame_id 空则用 base_link）
发布: /target_pose_status std_msgs/String（accepted / succeeded / failed: ...）

orientation 全 0 表示只约束位置、姿态自由。
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    OrientationConstraint,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String

SUCCESS = 1  # MoveItErrorCodes.SUCCESS


class TargetPoseBridge(Node):
    def __init__(self):
        super().__init__("target_pose_bridge")

        self.group = self.declare_parameter("planning_group", "arm").value
        self.ee_link = self.declare_parameter("ee_link", "link6").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.pos_tol = float(self.declare_parameter("position_tolerance", 0.005).value)
        self.ori_tol = float(self.declare_parameter("orientation_tolerance", 0.1).value)
        self.vel_scale = float(self.declare_parameter("velocity_scaling", 0.3).value)
        self.acc_scale = float(self.declare_parameter("acceleration_scaling", 0.3).value)
        self.plan_time = float(self.declare_parameter("planning_time", 5.0).value)
        self.attempts = int(self.declare_parameter("planning_attempts", 10).value)

        group = ReentrantCallbackGroup()
        self.status_pub = self.create_publisher(String, "target_pose_status", 10)
        self.create_subscription(
            PoseStamped, "target_pose", self._on_target, 10, callback_group=group
        )
        self.client = ActionClient(self, MoveGroup, "move_action", callback_group=group)
        self.busy = False
        self.get_logger().info(f"target_pose_bridge ready ({self.group}/{self.ee_link})")

    def _status(self, text):
        self.status_pub.publish(String(data=text))
        self.get_logger().info(f"[status] {text}")

    def _goal(self, msg):
        frame = msg.header.frame_id or self.base_frame
        pose = msg.pose

        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = self.group
        req.num_planning_attempts = self.attempts
        req.allowed_planning_time = self.plan_time
        req.max_velocity_scaling_factor = self.vel_scale
        req.max_acceleration_scaling_factor = self.acc_scale
        req.start_state.is_diff = True

        c = Constraints()

        # 位置约束：一个 2*tol 的立方体
        pos = PositionConstraint()
        pos.header.frame_id = frame
        pos.link_name = self.ee_link
        region = BoundingVolume()
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [2.0 * self.pos_tol] * 3
        region.primitives.append(box)
        region.primitive_poses.append(pose)
        pos.constraint_region = region
        pos.weight = 1.0
        c.position_constraints.append(pos)

        # 姿态约束：四元数全 0 时跳过（姿态自由）
        q = pose.orientation
        if math.sqrt(q.x ** 2 + q.y ** 2 + q.z ** 2 + q.w ** 2) > 1e-6:
            ori = OrientationConstraint()
            ori.header.frame_id = frame
            ori.link_name = self.ee_link
            ori.orientation = q
            ori.absolute_x_axis_tolerance = self.ori_tol
            ori.absolute_y_axis_tolerance = self.ori_tol
            ori.absolute_z_axis_tolerance = self.ori_tol
            ori.weight = 1.0
            c.orientation_constraints.append(ori)

        req.goal_constraints.append(c)
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        return goal

    def _on_target(self, msg):
        if self.busy:
            self._status("busy: 上一个目标还没执行完")
            return
        if not self.client.wait_for_server(timeout_sec=3.0):
            self._status("failed: move_action 服务不可用")
            return
        self.busy = True
        self._status("accepted: 已发给 move_group")
        self.client.send_goal_async(self._goal(msg)).add_done_callback(
            self._on_response
        )

    def _on_response(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.busy = False
            self._status("rejected: move_group 拒绝了目标")
            return
        self._status("planning")
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        self.busy = False
        result = future.result()
        if result is None:
            self._status("failed: 没有收到结果")
            return
        code = result.result.error_code.val
        self._status("succeeded" if code == SUCCESS else f"failed: code={code}")


def main(args=None):
    rclpy.init(args=args)
    node = TargetPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
