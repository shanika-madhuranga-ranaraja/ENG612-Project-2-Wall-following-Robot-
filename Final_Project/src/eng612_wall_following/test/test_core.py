"""Meaningful offline checks; no ROS installation required."""
import math
from types import SimpleNamespace
import unittest

from eng612_wall_following.core import Config, PID, WallController as ProductionWallController, observe, wrap


def arena_scan(x=-0.4, y=-0.7, yaw=0.0, start=-math.pi, count=360):
    ranges = []
    step = 2 * math.pi / count
    for i in range(count):
        a = yaw + start + i * step
        dx, dy = math.cos(a), math.sin(a)
        hits = []
        if abs(dx) > 1e-10:
            hits.extend(t for wall in [-1.0, 1.0] if (t := (wall - x) / dx) > 0)
        if abs(dy) > 1e-10:
            hits.extend(t for wall in [-1.0, 1.0] if (t := (wall - y) / dy) > 0)
        ranges.append(min(hits))
    return SimpleNamespace(ranges=ranges, angle_min=start, angle_increment=step,
                           range_min=0.05, range_max=10.0,
                           test_yaw=yaw, test_position=(x, y))


class WallController(ProductionWallController):
    """Inject the ray-caster's ground-truth odometry for offline regressions."""
    def step(self, scan, dt):
        return super().step(scan, dt, yaw=scan.test_yaw, position=scan.test_position)


def force_front(scan, distance):
    for i in range(len(scan.ranges)):
        angle = scan.angle_min + i * scan.angle_increment
        if abs(wrap(angle)) <= math.radians(20):
            scan.ranges[i] = distance
    return scan


class CoreTests(unittest.TestCase):
    def test_target_and_scan_wrapping(self):
        for start in [0.0, -math.pi]:
            o = observe(arena_scan(start=start), Config())
            self.assertAlmostEqual(o.distance, .3, places=3)
            self.assertAlmostEqual(o.predicted, .3, places=3)

    def test_right_wall_steering_sign(self):
        close = WallController(Config()).step(arena_scan(y=-0.8), .1)
        far = WallController(Config()).step(arena_scan(y=-0.6), .1)
        self.assertGreater(close.angular, 0)
        self.assertLess(far.angular, 0)

    def test_left_wall_steering_sign(self):
        cfg = Config(wall_side='left')
        self.assertLess(WallController(cfg).step(arena_scan(y=0.8), .1).angular, 0)
        self.assertGreater(WallController(cfg).step(arena_scan(y=0.6), .1).angular, 0)

    def test_lookahead_detects_approaching_wall(self):
        o = observe(arena_scan(yaw=-.15), Config())
        self.assertLess(o.predicted, o.distance)

    def test_inside_corner_turn_direction(self):
        cmd = WallController(Config()).step(arena_scan(x=0.75), .1)
        self.assertEqual(cmd.state, 'TURN_90')
        self.assertEqual(cmd.linear, 0)
        self.assertGreater(cmd.angular, 0)

    def test_invalid_front_and_wall_stop(self):
        scan = arena_scan()
        scan.ranges = [math.inf] * 360
        cmd = WallController(Config()).step(scan, .1)
        self.assertEqual((cmd.linear, cmd.angular), (0, 0))
        scan = arena_scan()
        for i in range(360):
            a = scan.angle_min + i * scan.angle_increment
            if -1.65 < a < -.7:
                scan.ranges[i] = math.nan
        self.assertEqual(WallController(Config()).step(scan, .1).state, 'INVALID_WALL')

    def test_clearance_veto_and_bad_dt(self):
        cmd = WallController(Config()).step(arena_scan(y=-0.9), .1)
        self.assertEqual(cmd.state, 'TOO_CLOSE')
        cmd = WallController(Config()).step(arena_scan(), 0)
        self.assertEqual(cmd.linear, 0)
        self.assertEqual(cmd.angular, 0)

    def test_range_min_is_not_discarded(self):
        scan = arena_scan()
        scan.ranges[180] = scan.range_min
        self.assertEqual(WallController(Config()).step(scan, .1).state, 'TOO_CLOSE')

    def test_dual_pid_reduces_speed(self):
        scan = arena_scan(y=-0.6)
        single = WallController(Config(mode='single')).step(scan, .1)
        dual = WallController(Config(mode='dual')).step(scan, .1)
        self.assertLess(dual.linear, single.linear)
        self.assertGreater(dual.linear, 0)

    def test_antiwindup(self):
        pid = PID(10, 2, 0, -1, 1)
        for _ in range(200):
            self.assertEqual(pid.step(2, .1), 1)
        self.assertEqual(pid.integral, 0)
        self.assertLess(pid.step(-.01, .1), 0)

    def test_turn_timeout_stays_stopped_until_reset(self):
        controller = WallController(Config(turn_timeout=0.5))
        scan = arena_scan(x=0.75)
        for _ in range(10):
            cmd = controller.step(scan, .1)
        self.assertEqual(cmd.state, 'TURN_TIMEOUT')
        self.assertEqual(controller.step(arena_scan(), .1).linear, 0)
        controller.reset()
        self.assertGreater(controller.step(arena_scan(), .1).linear, 0)

    def test_no_motion_without_odometry(self):
        controller = ProductionWallController(Config())
        cmd = controller.step(arena_scan(), .1)
        self.assertEqual((cmd.state, cmd.linear, cmd.angular), ('WAIT_ODOM', 0, 0))

    def test_turn_continues_when_front_clears_before_90_degrees(self):
        controller = WallController(Config(front_turn=.45))
        controller.step(arena_scan(x=.6), .1)
        target = controller.turn_target_yaw
        cmd = controller.step(arena_scan(x=.6, yaw=math.radians(75)), .1)
        self.assertGreater(cmd.observation.front, controller.cfg.front_release)
        self.assertEqual(cmd.state, 'TURN_90')
        self.assertEqual(controller.turn_target_yaw, target)
        self.assertGreater(cmd.angular, 0)
        self.assertEqual(cmd.linear, 0)

    def test_90_degree_target_wraps_correctly_in_both_directions(self):
        for side, initial, expected in [('right', 170, -100), ('left', -170, 100)]:
            with self.subTest(side=side):
                controller = WallController(Config(wall_side=side))
                first = force_front(arena_scan(x=0, y=0, yaw=math.radians(initial)), .25)
                cmd = controller.step(first, .1)
                self.assertEqual(cmd.state, 'TURN_90')
                self.assertAlmostEqual(math.degrees(controller.turn_target_yaw), expected)
                self.assertGreater(cmd.angular, 0) if side == 'right' else self.assertLess(cmd.angular, 0)
                cmd = controller.step(arena_scan(x=0, y=0, yaw=math.radians(expected)), .1)
                self.assertEqual((cmd.state, cmd.linear, cmd.angular), ('TURN_DONE', 0, 0))

    def test_corner_is_not_triggered_twice_and_rearms_after_progress(self):
        controller = WallController(Config())
        controller.step(arena_scan(x=.75), .1)
        cmd = controller.step(arena_scan(x=.75, yaw=math.pi/2), .1)
        self.assertEqual(cmd.state, 'TURN_DONE')
        cmd = controller.step(arena_scan(x=.75, yaw=math.pi/2), .1)
        self.assertEqual(cmd.state, 'FOLLOW')
        self.assertFalse(controller.corner_armed)
        cmd = controller.step(force_front(arena_scan(x=.75, yaw=math.pi/2), .25), .1)
        self.assertEqual(cmd.state, 'CORNER_BLOCKED')
        self.assertFalse(controller.turning)
        self.assertIsNone(controller.turn_target_yaw)
        cmd = controller.step(arena_scan(x=.75, y=-.55, yaw=math.pi/2), .1)
        self.assertEqual(cmd.state, 'FOLLOW')
        self.assertTrue(controller.corner_armed)
        cmd = controller.step(arena_scan(x=.75, y=.75, yaw=math.pi/2), .1)
        self.assertEqual(cmd.state, 'TURN_90')

    def test_pause_or_invalid_scan_preserves_existing_turn_target(self):
        controller = WallController(Config())
        controller.step(arena_scan(x=.75), .1)
        target = controller.turn_target_yaw
        controller.pause()
        scan = arena_scan(x=.75, yaw=math.radians(30))
        scan.ranges = [math.inf] * len(scan.ranges)
        self.assertEqual(controller.step(scan, .1).state, 'INVALID_FRONT')
        self.assertEqual(controller.turn_target_yaw, target)
        cmd = controller.step(arena_scan(x=.75, yaw=math.radians(30)), .1)
        self.assertEqual(cmd.state, 'TURN_90')
        self.assertEqual(controller.turn_target_yaw, target)

    def test_two_laps_ideal_kinematics(self):
        # Ideal point-lidar/unicycle regression, NOT a Gazebo or hardware test.
        # Integrates scan-based closed-loop response through repeated corners.
        for mode, side in [('single', 'right'), ('dual', 'right'), ('single', 'left'), ('dual', 'left')]:
            with self.subTest(mode=mode, side=side):
                controller = WallController(Config(mode=mode, wall_side=side))
                x, y, yaw = (-.4, -.7, 0.0) if side == 'right' else (.4, -.7, math.pi)
                total_turn = 0.0
                completed_corners = 0
                for _ in range(4000):
                    cmd = controller.step(arena_scan(x, y, yaw, count=180), .1)
                    self.assertIn(cmd.state, ['FOLLOW', 'TURN_90', 'TURN_DONE'])
                    completed_corners += cmd.state == 'TURN_DONE'
                    x += cmd.linear * math.cos(yaw) * .1
                    y += cmd.linear * math.sin(yaw) * .1
                    yaw += cmd.angular * .1
                    total_turn += cmd.angular * .1
                    self.assertLess(abs(x), 0.86)
                    self.assertLess(abs(y), 0.86)
                self.assertGreater(abs(total_turn), 4 * math.pi)
                self.assertGreaterEqual(completed_corners, 8)


if __name__ == '__main__':
    unittest.main()
