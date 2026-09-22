#!/usr/bin/env python3
"""Odometry from the wheels, the way the real robot does it."""

import math

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

POSE_COVARIANCE = [0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.001]
TWIST_COVARIANCE = list(POSE_COVARIANCE)


def yaw_to_quaternion(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def shortest(step):
    """A joint-angle step folded into (-pi, pi]."""
    return math.atan2(math.sin(step), math.cos(step))


class WheelOdometry(Node):

    def __init__(self):
        super().__init__('wheel_odometry')

        self.declare_parameter('wheel_radius', 0.033)
        self.declare_parameter('wheel_separation', 0.160)
        self.declare_parameter('left_joint', 'wheel_left_joint')
        self.declare_parameter('right_joint', 'wheel_right_joint')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('publish_tf', True)

        p = self.get_parameter
        self.radius = p('wheel_radius').value
        self.separation = p('wheel_separation').value
        self.left = p('left_joint').value
        self.right = p('right_joint').value
        self.odom_frame = p('odom_frame_id').value
        self.base_frame = p('base_frame_id').value

        self.x = self.y = self.yaw = 0.0
        self._last_raw = {}
        self._last_stamp = None

        self.pub = self.create_publisher(
            Odometry, 'odom',
            QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
        self.tf = TransformBroadcaster(self) if p('publish_tf').value else None
        self.create_subscription(JointState, 'joint_states', self.on_joints, 50)

        self.get_logger().info(
            f'wheel odometry: r={self.radius} separation={self.separation}, '
            f'{self.odom_frame} -> {self.base_frame}'
            f'{"" if self.tf else " (no tf)"}')

    def on_joints(self, msg):
        angles = {}
        for i, name in enumerate(msg.name):
            if name in (self.left, self.right) and i < len(msg.position):
                angles[name] = msg.position[i]
        if len(angles) != 2:
            return

        stamp = msg.header.stamp
        now = stamp.sec + stamp.nanosec * 1e-9

        if self._last_stamp is None:
            self._last_raw = dict(angles)
            self._last_stamp = now
            return

        dt = now - self._last_stamp
        if dt <= 0.0:
            self._last_raw = dict(angles)
            self._last_stamp = now
            return

        dl = shortest(angles[self.left] - self._last_raw[self.left]) * self.radius
        dr = shortest(angles[self.right] - self._last_raw[self.right]) * self.radius
        self._last_raw = dict(angles)
        self._last_stamp = now

        d_centre = (dl + dr) / 2.0
        d_yaw = (dr - dl) / self.separation

        self.x += d_centre * math.cos(self.yaw + d_yaw / 2.0)
        self.y += d_centre * math.sin(self.yaw + d_yaw / 2.0)
        self.yaw = math.atan2(math.sin(self.yaw + d_yaw),
                              math.cos(self.yaw + d_yaw))

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = yaw_to_quaternion(self.yaw)
        odom.pose.covariance = POSE_COVARIANCE
        odom.twist.twist.linear.x = d_centre / dt
        odom.twist.twist.angular.z = d_yaw / dt
        odom.twist.covariance = TWIST_COVARIANCE
        self.pub.publish(odom)

        if self.tf is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation = odom.pose.pose.orientation
            self.tf.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometry()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
