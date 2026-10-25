"""Display-only odometry conversion; does not affect motion control."""
import math


def planar_pose(message):
    """Return x, y, yaw in radians, and the actual odometry parent frame.

    Reject missing/nonfinite poses and zero-norm quaternions rather than
    displaying a plausible zero position. Normalise quaternion before yaw.
    """
    p, q = message.pose.pose.position, message.pose.pose.orientation
    values = (p.x, p.y, q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in values):
        return None
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if norm < 1e-8:
        return None
    x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return (float(p.x), float(p.y), yaw, message.header.frame_id or 'unknown')


def number(value, unit='m', signed=False):
    if value is None or not math.isfinite(value):
        return '--'
    return (f'{value:+.3f}' if signed else f'{value:.3f}') + unit
