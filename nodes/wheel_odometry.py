#!/usr/bin/env python3
"""Odometry from the wheels, the way the real robot does it.

Isaac Sim's stock odometry is `IsaacComputeOdometry`, which reads the chassis
prim -- so `/odom` carried the robot's TRUE pose. That is not what `/odom` means
anywhere else. `turtlebot3_node` integrates wheel encoders on the real burger
and `gazebo_ros_diff_drive` integrates wheel rotation in Gazebo; both drift, and
Nav2's whole job above `odom` is to correct that drift.

Publishing the truth there made Isaac Sim the odd backend out in a repository
whose entire claim is that the three are interchangeable, and it made
localisation unrealistically easy on exactly one of them: AMCL had nothing to
correct. It also hid a real phenomenon. The wheels of this robot slip about 7%
in a pivot at `wz = 0.5` and 0.5% driving straight (measured, `docs/status.md`
in tb3_sim2real) -- a burger pivoting on two wheels and a plastic skid does
exactly that, and ground-truth odometry reports none of it.

So this node consumes `/joint_states` and publishes `/odom` plus the
`odom -> base_footprint` transform, and the simulator's own chassis-prim
odometry moves to `/ground_truth/odom` -- the same topic the Gazebo backend
publishes its P3D pose on, so the two simulators finally agree about what each
name means.

Why a ROS 2 node and not an OmniGraph node: there is no stock OmniGraph node
that integrates differential-drive odometry. NVIDIA ships `IsaacComputeOdometry`
and nothing encoder-based, which is why the reference graph -- adopted here
verbatim, correctly, under this package's "prefer upstream's own examples" rule
-- reads the prim. A custom graph node would have to be written in Kit's Python
against an unstable node API, to do arithmetic that belongs in ROS. The real
robot computes its odometry in a ROS node too.

POSITION, not velocity. A continuous joint's reported angle may accumulate or
may be folded into [-pi, pi]; this unwraps per sample, the same way
`tb3_bringup`'s `drive_test` does. Integrating the velocity field instead would
be integrating the chatter the 240 Hz sub-step rate exists to suppress, and
would drift differently from a real encoder, which counts ticks.
"""

import math

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

# turtlebot3_node's values, so a consumer sees the same numbers on every
# backend. They are a statement that odometry is trusted in x and yaw and not
# at all in y, z, roll or pitch, rather than a calibrated measurement.
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
    """A joint-angle step folded into (-pi, pi].

    Guards the wrap of a folded continuous joint. At the rates this robot
    reaches, one sample is far below pi, so the fold is never ambiguous.
    """
    return math.atan2(math.sin(step), math.cos(step))


class WheelOdometry(Node):

    def __init__(self):
        super().__init__('wheel_odometry')

        # Defaults are the burger's, and are the same numbers
        # turtlebot3_gazebo's model.sdf states. They are parameters so that a
        # waffle needs no code change, not so that they can be tuned: a wheel
        # radius that disagrees with the URDF is a silent scale error in every
        # measurement this repository takes.
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

        # RELIABLE to match every other odometry publisher in this stack, so a
        # QoS mismatch is an error rather than a silently empty subscription.
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

        # The first sample only establishes a reference. The odom frame is
        # defined by wherever the robot was then -- the same convention as
        # turtlebot3_node and gazebo_ros_diff_drive, so a run that starts at
        # the manifest spawn reports a pose relative to it on all three
        # backends.
        if self._last_stamp is None:
            self._last_raw = dict(angles)
            self._last_stamp = now
            return

        dt = now - self._last_stamp
        if dt <= 0.0:
            # A simulator restart steps the clock backwards. Re-reference
            # rather than integrating a negative interval.
            self._last_raw = dict(angles)
            self._last_stamp = now
            return

        dl = shortest(angles[self.left] - self._last_raw[self.left]) * self.radius
        dr = shortest(angles[self.right] - self._last_raw[self.right]) * self.radius
        self._last_raw = dict(angles)
        self._last_stamp = now

        d_centre = (dl + dr) / 2.0
        d_yaw = (dr - dl) / self.separation

        # Midpoint integration: advance along the heading the robot held in the
        # middle of the step, not at its start. Over an arc the difference is
        # second order, and it is free.
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
        # launch stops a node with SIGTERM, which rclpy surfaces as
        # ExternalShutdownException. Uncaught it prints a traceback on every
        # ordinary shutdown, which is noise in the one place noise hides real
        # failures.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
