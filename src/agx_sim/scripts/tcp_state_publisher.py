#!/usr/bin/env python3
"""发布夹爪 TCP 在基坐标系下的位姿。

订阅: 无（只查 TF）
发布: /arm_tcp_state  geometry_msgs/PoseStamped

TCP 取在夹爪两指指尖的中点（默认就是 gripper_base 沿 +Z 走 0.1358 m 处），
姿态沿用 gripper_base。真实夹爪想改抓取点，用参数 tcp_z 调偏移即可。
"""

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class TcpStatePublisher(Node):
    def __init__(self):
        super().__init__("tcp_state_publisher")

        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.tcp_parent = self.declare_parameter("tcp_parent", "gripper_base").value
        self.tcp_z = float(self.declare_parameter("tcp_z", 0.1358).value)
        self.rate = float(self.declare_parameter("publish_rate", 50.0).value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PoseStamped, "arm_tcp_state", 10)
        self.create_timer(1.0 / self.rate, self._tick)

    def _tick(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.tcp_parent, rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except Exception:
            return  # TF 还没准备好，跳过这一帧

        t, q = tf.transform.translation, tf.transform.rotation

        # 把局部偏移 (0,0,tcp_z) 用父坐标系姿态旋到基坐标系
        # 四元数转旋转矩阵，避免额外依赖
        x, y, z, w = q.x, q.y, q.z, q.w
        m = [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]

        msg = PoseStamped()
        msg.header.stamp = tf.header.stamp
        msg.header.frame_id = self.base_frame
        msg.pose.position.x = t.x + m[0][2] * self.tcp_z
        msg.pose.position.y = t.y + m[1][2] * self.tcp_z
        msg.pose.position.z = t.z + m[2][2] * self.tcp_z
        msg.pose.orientation = q
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TcpStatePublisher()
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
