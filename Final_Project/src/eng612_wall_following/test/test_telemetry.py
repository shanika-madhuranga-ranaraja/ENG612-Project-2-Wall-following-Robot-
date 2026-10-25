import math
from types import SimpleNamespace as S
import unittest

from eng612_wall_following.telemetry import planar_pose


def odom(q, x=1.2, y=-.4):
    return S(header=S(frame_id='odom'), pose=S(pose=S(
        position=S(x=x, y=y), orientation=S(x=q[0], y=q[1], z=q[2], w=q[3]))))


class OdometryTests(unittest.TestCase):
    def test_position_and_frame_are_preserved(self):
        self.assertEqual(planar_pose(odom((0, 0, 0, 1))), (1.2, -.4, 0.0, 'odom'))

    def test_yaw_direction_and_quaternion_normalisation(self):
        for theta in (-2.5, -math.pi/2, 0, math.pi/2, 2.5):
            msg = odom((0, 0, 3*math.sin(theta/2), 3*math.cos(theta/2)))
            self.assertAlmostEqual(planar_pose(msg)[2], theta, places=10)

    def test_invalid_pose_is_unknown_instead_of_zero(self):
        self.assertIsNone(planar_pose(odom((0, 0, 0, 0))))
        self.assertIsNone(planar_pose(odom((0, 0, math.nan, 1))))
        self.assertIsNone(planar_pose(odom((0, 0, 0, 1), x=math.inf)))
