"""Wheel geometry, in one place.

Three consumers need these numbers and they must not disagree: the
DifferentialController in the simulator's OmniGraph, which turns /cmd_vel into
wheel velocities; nodes/wheel_odometry.py, which turns wheel angles back into a
pose; and anything comparing the two. A wheel radius that differs between the
forward and the inverse map is a scale error that looks exactly like slip.

Deliberately importing nothing. The launch file loads this by path so that it
can pass the right numbers for the model without importing the simulator
runtime, which pulls in Kit.

The values are turtlebot3_gazebo's model.sdf, so the two simulators start from
the same geometry; that SDF and turtlebot3_description's URDF agree.
"""

WHEEL_JOINTS = ['wheel_left_joint', 'wheel_right_joint']

WHEELS = {
    'burger': {'separation': 0.160, 'radius': 0.033},
    'waffle': {'separation': 0.287, 'radius': 0.033},
    'waffle_pi': {'separation': 0.287, 'radius': 0.033},
}
