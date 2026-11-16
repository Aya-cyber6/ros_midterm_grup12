"""Sensor validation and motion decisions independent of ROS."""

import math


def front_distance(ranges, angle_min, angle_increment, range_min, range_max):
    """Return front clearance, or None when no usable front sample exists."""
    if not math.isfinite(angle_increment) or angle_increment <= 0:
        return None
    samples = []
    for index, value in enumerate(ranges):
        angle = angle_min + index * angle_increment
        angle = math.atan2(math.sin(angle), math.cos(angle))
        if abs(angle) <= math.radians(20):
            if value == math.inf:
                samples.append(range_max)
            elif math.isfinite(value) and range_min <= value <= range_max:
                samples.append(value)
    return min(samples) if samples else None


def range_distance(value, minimum, maximum):
    """Validate a single sonar measurement."""
    if value == math.inf:
        return maximum
    if value == -math.inf:
        return 0.0
    if math.isfinite(value) and minimum <= value <= maximum:
        return value
    return None


def motion(lidar, sonar, lidar_age, sonar_age, timeout=0.5):
    """Stop on missing/stale data; otherwise turn at obstacles or drive."""
    if (lidar is None or sonar is None or
            lidar_age > timeout or sonar_age > timeout):
        return 0.0, 0.0
    if lidar < 0.5 or sonar < 0.4:
        return 0.0, 0.5
    return 0.3, 0.0
