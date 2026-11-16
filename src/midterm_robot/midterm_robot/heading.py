"""IMU heading conversion and feedback-controlled obstacle turns."""

import math

from midterm_robot.safety import motion


def angle_difference(target, current):
    """Return the shortest signed angular difference in radians."""
    return math.atan2(math.sin(target - current), math.cos(target - current))


def quaternion_yaw(x, y, z, w):
    """Return yaw from a valid normalized quaternion, or None."""
    if not all(math.isfinite(v) for v in (x, y, z, w)):
        return None
    norm = math.hypot(x, y, z, w)
    if norm < 1e-12 or not math.isfinite(norm):
        return None
    x, y, z, w = (v / norm for v in (x, y, z, w))
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class HeadingController:
    """Turn by a configured angle relative to the obstacle detection heading."""

    def __init__(self, turn_angle_deg=45.0):
        if not math.isfinite(turn_angle_deg) or not 5 <= abs(turn_angle_deg) <= 170:
            raise ValueError('turn_angle_deg must be between 5 and 170 in magnitude')
        self.turn_angle = math.radians(turn_angle_deg)
        self.tolerance = math.radians(3.0)
        self.target_yaw = None

    def command(self, lidar, sonar, yaw, lidar_age, sonar_age, imu_age):
        """Require three fresh sensors and preserve a turn target until reached."""
        linear, angular = motion(lidar, sonar, lidar_age, sonar_age)
        if (yaw is None or not math.isfinite(yaw) or imu_age > 0.5 or
                (linear == 0.0 and angular == 0.0)):
            self.target_yaw = None
            return 0.0, 0.0
        if self.target_yaw is None and angular != 0.0:
            self.target_yaw = angle_difference(yaw + self.turn_angle, 0.0)
        if self.target_yaw is None:
            return linear, 0.0
        error = angle_difference(self.target_yaw, yaw)
        if abs(error) <= self.tolerance:
            self.target_yaw = None
            # Stop this tick and check the obstacle again on the next tick.
            return 0.0, 0.0
        speed = min(0.5, max(0.1, 1.5 * abs(error)))
        return 0.0, math.copysign(speed, error)
