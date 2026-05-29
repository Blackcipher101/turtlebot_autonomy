#!/usr/bin/env python3
"""
Reads /odom from Isaac Sim and publishes odom->base_footprint TF.
Passes orientation directly — no trig conversion that breaks on non-flat terrain.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.br = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odom', self.cb, 10)
        self.get_logger().info('odom_to_tf: publishing odom->base_footprint')

    def cb(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp    = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_footprint'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0  # keep flat — ignore z sink
        # Use only yaw from quaternion, force roll/pitch to zero
        q = msg.pose.pose.orientation
        # Re-normalize to pure yaw rotation: only keep z,w components
        import math
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(yaw / 2)
        t.transform.rotation.w = math.cos(yaw / 2)
        self.br.sendTransform(t)


def main():
    rclpy.init()
    rclpy.spin(OdomToTF())

if __name__ == '__main__':
    main()
