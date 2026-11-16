# Group 12 mobile robot

ROS 2 with modern Gazebo Sim (`gz sim`). The narrow front sensor approximates sonar with GPU lidar; acoustic propagation is not simulated.

## Run on Ubuntu

Start from the repository root with ROS 2 sourced and a matching ROS/Gazebo installation (for example Jazzy/Harmonic), rosdep and colcon installed:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select midterm_robot
source install/setup.bash
ros2 launch midterm_robot sim.launch.py
```

The controller starts automatically and waits for BOTH sensors. It turns for nearby obstacles and stops on missing, invalid or over 0.5 s old data. Do not launch a second controller. The optional sonar_safety_node only logs warnings.

RViz is disabled by default. Add `rviz:=true` to the launch command to enable it. GPU lidar and the camera still require functioning Gazebo graphics; VM graphics compatibility is not established by these changes.

## Simulator verification

In a second sourced terminal, inspect each command separately (Ctrl+C to stop):

```bash
ros2 topic hz /scan
ros2 topic hz /sonar/range
ros2 topic hz /imu
ros2 topic echo /cmd_vel
```

Confirm the robot spawns and turns before touching the red box. Interrupt either sensor stream and check that velocity becomes zero within approximately 0.5 s. A paused simulator cannot physically move or stop until physics resumes.

## Tests

Without ROS:

```bash
PYTHONPATH=src/midterm_robot python3 -m unittest discover -s src/midterm_robot/test -p test_safety.py -v
```

With ROS test dependencies installed, use `colcon test --packages-select midterm_robot` and `colcon test-result --verbose`.

Pure tests cover sensor validity, front-angle selection, missing/stale data and motion decisions. Configuration checks cover bridge declarations, sonar attachment and world sensor systems. Actual spawn, sensor output, rendering and collision behavior require simulator verification.
