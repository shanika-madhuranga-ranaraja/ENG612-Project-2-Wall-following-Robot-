"""ROS-independent scan processing and control; distances are from the lidar.

Body axes: +x forward, +y left; positive angular velocity turns left.
Initial gains/dimensions are design choices, not measured laboratory results.
"""
from dataclasses import dataclass
import math
from statistics import median


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class PID:
    def __init__(self, kp, ki, kd, lower, upper, integral_limit=0.5, tau=0.15):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.lower, self.upper = lower, upper
        self.integral_limit, self.tau = integral_limit, tau
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.previous = None
        self.derivative = 0.0

    def step(self, error, dt):
        if not math.isfinite(error) or not 0.0 < dt <= 1.0:
            self.reset()
            return 0.0
        raw_d = 0.0 if self.previous is None else (error - self.previous) / dt
        weight = dt / (self.tau + dt)
        self.derivative += weight * (raw_d - self.derivative)
        candidate = clamp(self.integral + error * dt,
                          -self.integral_limit, self.integral_limit)
        raw = self.kp * error + self.ki * candidate + self.kd * self.derivative
        # Conditional integration: retain integration only if it is not driving
        # the output further into saturation. Integral state is bounded too.
        if not ((raw > self.upper and error > 0) or
                (raw < self.lower and error < 0)):
            self.integral = candidate
        self.previous = error
        return clamp(self.kp * error + self.ki * self.integral +
                     self.kd * self.derivative, self.lower, self.upper)


@dataclass
class Config:
    mode: str = 'single'
    wall_side: str = 'right'
    target_distance: float = 0.30
    base_speed: float = 0.10
    min_speed: float = 0.025
    max_angular: float = 0.8
    lookahead: float = 0.12
    angular_kp: float = 2.0
    angular_ki: float = 0.02
    angular_kd: float = 0.10
    linear_kp: float = 0.35
    linear_ki: float = 0.008
    linear_kd: float = 0.015
    front_turn: float = 0.30
    front_release: float = 0.58
    emergency_distance: float = 0.15
    turn_speed: float = 0.55
    turn_angle_deg: float = 90.0
    turn_tolerance_deg: float = 2.0
    turn_min_speed: float = 0.08
    turn_heading_kp: float = 1.5
    turn_timeout: float = 12.0
    corner_rearm_distance: float = 0.12
    slowdown_distance: float = 0.80
    speed_error_tolerance: float = 0.02
    wall_lost_distance: float = 1.0
    scan_yaw_offset: float = 0.0


@dataclass
class Observation:
    distance: float
    predicted: float
    front: float
    nearest: float
    front_valid: bool


def sector(scan, centre, halfwidth, offset=0.0):
    """Return finite valid samples selected by angle, never fixed indices.

    Conservative policy: inf/NaN/zero/out-of-range readings are unknown.
    This enclosed-arena controller does not treat an all-inf sector as free.
    """
    values = []
    for i, r in enumerate(scan.ranges):
        angle = scan.angle_min + i * scan.angle_increment + offset
        if abs(wrap(angle - centre)) <= halfwidth:
            if math.isfinite(r) and 0.0 < r and scan.range_min <= r <= scan.range_max:
                values.append(float(r))
    return values


def observe(scan, cfg):
    side = -1 if cfg.wall_side == 'right' else 1
    beam = math.radians(3.0)
    b_values = sector(scan, side * math.pi / 2, beam, cfg.scan_yaw_offset)
    a_values = sector(scan, side * math.pi / 4, beam, cfg.scan_yaw_offset)
    front_values = sector(scan, 0.0, math.radians(20), cfg.scan_yaw_offset)
    # Whole scan veto, so a very close side obstacle also causes a stop.
    all_values = sector(scan, 0.0, math.pi, cfg.scan_yaw_offset)
    b = median(b_values) if len(b_values) >= 2 else math.nan
    a = median(a_values) if len(a_values) >= 2 else math.nan
    distance, predicted = b, b
    if math.isfinite(a) and math.isfinite(b):
        theta = math.pi / 4
        alpha = math.atan2(a * math.cos(theta) - b, a * math.sin(theta))
        distance = b * math.cos(alpha)
        predicted = distance + cfg.lookahead * math.sin(alpha)
    return Observation(distance, predicted,
                       min(front_values) if front_values else math.nan,
                       min(all_values) if all_values else math.nan,
                       len(front_values) >= 3)


@dataclass
class Command:
    linear: float
    angular: float
    state: str
    observation: Observation
    error: float = math.nan
    slowdown_error: float = math.nan


class WallController:
    def __init__(self, cfg):
        if cfg.mode not in ('single', 'dual') or cfg.wall_side not in ('right', 'left'):
            raise ValueError('mode: single/dual; wall_side: right/left')
        if not (0 <= cfg.min_speed < cfg.base_speed <= 0.22 and
                0 < cfg.turn_speed <= cfg.max_angular and
                0 < cfg.emergency_distance < cfg.front_turn < cfg.front_release <
                cfg.slowdown_distance and
                cfg.emergency_distance < cfg.target_distance < cfg.wall_lost_distance):
            raise ValueError('Invalid speed/distance limits; inspect configuration')
        turn_values = (cfg.turn_angle_deg, cfg.turn_tolerance_deg, cfg.turn_min_speed,
                       cfg.turn_heading_kp, cfg.turn_timeout, cfg.corner_rearm_distance)
        if not (all(math.isfinite(v) for v in turn_values) and
                0 < cfg.turn_tolerance_deg < cfg.turn_angle_deg < 180 and
                0 < cfg.turn_min_speed <= cfg.turn_speed and
                cfg.turn_heading_kp > 0 and cfg.turn_timeout > 0 and
                cfg.corner_rearm_distance > 0):
            raise ValueError('Invalid odometry-based corner parameters')
        self.cfg = cfg
        self.angular_pid = PID(cfg.angular_kp, cfg.angular_ki, cfg.angular_kd,
                               -cfg.max_angular, cfg.max_angular)
        self.linear_pid = PID(cfg.linear_kp, cfg.linear_ki, cfg.linear_kd,
                              0.0, cfg.base_speed - cfg.min_speed)
        self.reset()

    def reset(self):
        self.fault = None
        self.turning = False
        self.turn_elapsed = 0.0
        self.turn_target_yaw = None
        self.corner_armed = True
        self.rearm_origin = None
        self.pause()

    def pause(self):
        # A scan/odometry interruption or enable-service pause must not discard
        # a partly completed corner and create a second 90-degree target.
        self.angular_pid.reset()
        self.linear_pid.reset()

    def stop(self, obs, state, latch=False):
        self.pause()
        if latch:
            self.fault = state
        return Command(0.0, 0.0, state, obs)

    def step(self, scan, dt, yaw=None, position=None):
        c = self.cfg
        obs = observe(scan, c)
        if self.fault:
            return Command(0.0, 0.0, self.fault, obs)
        if not 0.0 < dt <= 1.0:
            return self.stop(obs, 'BAD_DT')
        if math.isfinite(obs.nearest) and obs.nearest < c.emergency_distance:
            return self.stop(obs, 'TOO_CLOSE')
        if not obs.front_valid:
            return self.stop(obs, 'INVALID_FRONT')
        if (yaw is None or not math.isfinite(yaw) or position is None or
                len(position) != 2 or not all(math.isfinite(v) for v in position)):
            return self.stop(obs, 'WAIT_ODOM')

        if not self.corner_armed and not self.turning and self.rearm_origin is not None:
            travelled = math.hypot(position[0] - self.rearm_origin[0],
                                   position[1] - self.rearm_origin[1])
            if travelled >= c.corner_rearm_distance and obs.front > c.front_release:
                self.corner_armed = True
                self.rearm_origin = None

        if self.corner_armed and not self.turning and obs.front <= c.front_turn:
            self.turning = True
            self.corner_armed = False
            self.turn_elapsed = 0.0
            turn_sign = 1 if c.wall_side == 'right' else -1
            self.turn_target_yaw = wrap(yaw + turn_sign * math.radians(c.turn_angle_deg))
            self.pause()
        if self.turning:
            self.turn_elapsed += dt
            if self.turn_elapsed > c.turn_timeout:
                return self.stop(obs, 'TURN_TIMEOUT', latch=True)
            remaining = wrap(self.turn_target_yaw - yaw)
            if abs(remaining) <= math.radians(c.turn_tolerance_deg):
                self.turning = False
                self.turn_target_yaw = None
                self.rearm_origin = (float(position[0]), float(position[1]))
                self.pause()
                return Command(0.0, 0.0, 'TURN_DONE', obs)
            speed = clamp(c.turn_heading_kp * abs(remaining), c.turn_min_speed, c.turn_speed)
            return Command(0.0, math.copysign(speed, remaining), 'TURN_90', obs)

        # Never trigger another corner just because a noisy front measurement
        # drops again before the robot has left the previous corner. Stop and
        # report the obstruction if forward motion is not clear.
        if not self.corner_armed and obs.front <= c.front_turn:
            return self.stop(obs, 'CORNER_BLOCKED')
        if not math.isfinite(obs.predicted) or not math.isfinite(obs.distance):
            return self.stop(obs, 'INVALID_WALL')
        if obs.distance > c.wall_lost_distance:
            return self.stop(obs, 'WALL_LOST')
        error = obs.predicted - c.target_distance
        side = -1 if c.wall_side == 'right' else 1
        angular = side * self.angular_pid.step(error, dt)
        q = (abs(error) + max(0.0, c.slowdown_distance - obs.front)
             - c.speed_error_tolerance)
        if c.mode == 'single':
            linear = c.base_speed
        else:
            # Second PID regulates a composite clearance/error objective by
            # commanding a speed reduction. This is not an odometry speed servo.
            linear = c.base_speed - self.linear_pid.step(q, dt)
        return Command(linear, angular, 'FOLLOW', obs, error, q)
