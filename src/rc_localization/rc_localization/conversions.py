"""Between ROS messages and the planar poses of lib/frame_math."""

from geometry_msgs.msg import Quaternion

from rc_localization.lib.frame_math import Pose2D, quaternion_to_yaw, yaw_to_quaternion


def to_pose2d(position, orientation) -> Pose2D:
    """Pose2D from a ROS position or translation, and a quaternion."""
    q = orientation
    return Pose2D(position.x, position.y, quaternion_to_yaw(q.x, q.y, q.z, q.w))


def to_quaternion(yaw: float) -> Quaternion:
    x, y, z, w = yaw_to_quaternion(yaw)
    return Quaternion(x=x, y=y, z=z, w=w)
