"""Bounded front-bumper recovery using simulation time and IMU feedback."""

import math

from midterm_robot.heading import angle_difference
from midterm_robot.safety import motion


class CollisionRecovery:
    """Stop, reverse briefly, then turn left; abort unsafe or stuck recovery."""

    def __init__(self):
        self.state = 'idle'
        self.phase_start = None
        self.last_time = None
        self.last_contact_time = float('-inf')
        self.target_yaw = None
        self.reason = ''

    def block(self, reason):
        """Hold zero velocity until the controller is restarted."""
        self.state = 'blocked'
        self.target_yaw = None
        self.reason = reason
        return 0.0, 0.0

    def contact(self, now):
        """Record a nonempty contact message; return whether to stop now."""
        if not math.isfinite(now):
            self.block('Invalid simulation time')
            return True
        self.last_contact_time = now
        if self.state == 'idle':
            self.state = 'stopping'
            self.phase_start = now
            self.last_time = now
            return True
        if self.state == 'turning':
            self.block('Another collision during the recovery turn')
            return True
        # Initial contact can persist while reversing away. Do not restart
        # the timer on every contact message, which could cause endless backing.
        return False

    def command(self, now, lidar, sonar, yaw, lidar_age, sonar_age, imu_age):
        """Return a recovery command, or None to allow normal obstacle control."""
        if self.state == 'idle':
            return None
        if self.state == 'blocked':
            return 0.0, 0.0
        if not math.isfinite(now) or now < self.last_time:
            return self.block('Simulation time moved backwards or is invalid')
        self.last_time = now
        if (yaw is None or not math.isfinite(yaw) or imu_age > 0.5 or
                motion(lidar, sonar, lidar_age, sonar_age) == (0.0, 0.0)):
            return self.block('Required sensor data lost during recovery')

        elapsed = now - self.phase_start
        if self.state == 'stopping':
            if elapsed < 0.2:
                return 0.0, 0.0
            self.state = 'reversing'
            self.phase_start = now
            return -0.1, 0.0

        if self.state == 'reversing':
            if elapsed < 1.0:
                return -0.1, 0.0
            # Gazebo Contact publishes contact events, not an empty heartbeat.
            # Recent events at the end of backing mean contact may remain.
            if now - self.last_contact_time <= 0.2:
                return self.block('Bumper contact persists after reversing')
            self.state = 'turning'
            self.phase_start = now
            self.target_yaw = angle_difference(yaw + math.radians(45), 0.0)
            return 0.0, 0.0

        if self.state == 'turning':
            if elapsed >= 10.0:
                return self.block('Recovery turn timed out')
            error = angle_difference(self.target_yaw, yaw)
            if abs(error) <= math.radians(3):
                self.state = 'idle'
                self.target_yaw = None
                # Normal control checks the front again on the next tick.
                return 0.0, 0.0
            speed = min(0.5, max(0.1, 1.5 * abs(error)))
            return 0.0, math.copysign(speed, error)

        return self.block('Unknown recovery state')
