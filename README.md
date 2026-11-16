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

The controller starts automatically and waits for LiDAR, sonar AND valid IMU orientation. It turns for nearby obstacles and stops on missing, invalid or over 0.5 s old data. Do not launch a second controller. The optional sonar_safety_node only logs warnings.

RViz is disabled by default. Add `rviz:=true` to the launch command to enable it. GPU lidar and the camera still require functioning Gazebo graphics; VM graphics compatibility is not established by these changes.

## Heading deviation and obstacle turn angle

The first valid IMU orientation is the reference heading. The controller
publishes signed heading deviation in degrees on `/heading_deviation_deg`
and logs it once per second. Positive means left and negative means right.
The value wraps at +/-180 degrees; it is not a cumulative rotation counter.

Obstacle detection sets a fixed target 45 degrees to the left of the current
heading. The robot turns in place using IMU feedback, slows near the target,
and stops within 3 degrees of it. It then checks the obstacle again: a clear
front permits forward motion, while a remaining obstacle starts another turn.
Thus several 45-degree maneuvers can accumulate; this is not path planning or
a return to the initial route. Normal obstacle avoidance does not reverse;
the front-bumper recovery below is the only reverse maneuver.
Missing/invalid IMU orientation or any stale sensor stops the robot and cancels
the current target; a fresh obstacle check starts a new maneuver after recovery.

To select a different angle at startup:

```bash
ros2 launch midterm_robot sim.launch.py turn_angle_deg:=30.0
ros2 topic echo /heading_deviation_deg
```

The parameter accepts magnitudes from 5 to 170 degrees. Negative angles select
right turns. Changing it requires restarting the launch.

## Front-bumper collision recovery

A nonempty `/bumper_states` message immediately commands zero forward and
angular velocity and cancels any ordinary obstacle turn. The recovery sequence
then holds zero velocity for 0.2 seconds, reverses at 0.1 m/s for 1 second,
stops and turns 45 degrees to the left using IMU feedback (3-degree tolerance).
It checks the front again before allowing forward motion. The recovery angle
is always +45 degrees; `turn_angle_deg` changes only ordinary obstacle avoidance.

Recovery durations use simulation time, while a steady-clock timer checks
sensor freshness even when the simulation is paused. Backing is time-based,
not distance-controlled: 10 cm is the nominal displacement at full commanded
speed, not a measured or guaranteed distance. There is no rear obstacle sensor.

Repeated contact messages during the initial stop/backing do not restart the
reverse timer. Contact still present at the end of backing, another contact
during the recovery turn, invalid/stale required sensors, a clock reset or a
turn lasting over 10 seconds cause a stop that requires restarting the controller.
Gazebo Contact sends events on contact, not an empty heartbeat; a 0.2-second
contact-free interval is required before starting the recovery turn.

The bumper has a small physical inertia and its fixed joint is preserved so
the contact sensor references the actual `bumper_link_collision` in Gazebo.
This sensor covers the front bumper, not every possible body or wheel collision.

## Simulator verification

In a second sourced terminal, inspect each command separately (Ctrl+C to stop):

```bash
ros2 topic hz /scan
ros2 topic hz /sonar/range
ros2 topic hz /imu
ros2 topic echo /bumper_states
ros2 topic echo /cmd_vel
```

Confirm the robot spawns and turns before touching the red box. Interrupt any of the three sensor streams and check that velocity becomes zero within approximately 0.5 s. A paused simulator cannot physically move or stop until physics resumes.

Also test actual front-bumper contact: observe a zero command, a bounded
negative linear command, zero linear velocity with an IMU-measured +45-degree
turn, then a new front-clearance check. Recontact during the turn must stop it.
The collision recovery has been checked with simulated inputs, not in Gazebo.

## Tests

Without ROS:

```bash
PYTHONPATH=src/midterm_robot python3 -m unittest discover -s src/midterm_robot/test -p test_safety.py -v
```

With ROS test dependencies installed, use `colcon test --packages-select midterm_robot` and `colcon test-result --verbose`.

Pure tests cover sensor validity, front-angle selection, missing/stale data and motion decisions. Configuration checks cover bridge declarations, sonar attachment and world sensor systems. Actual spawn, sensor output, rendering and collision behavior require simulator verification.

Heading tests without ROS:

```bash
PYTHONPATH=src/midterm_robot python3 -m unittest discover -s src/midterm_robot/test -p test_heading.py -v
PYTHONPATH=src/midterm_robot python3 -m unittest discover -s src/midterm_robot/test -p test_recovery.py -v
PYTHONPATH=src/midterm_robot python3 -m unittest discover -s src/midterm_robot/test -p test_bumper_config.py -v
```

Verify measured turns and topic output in Gazebo before merging; the tests do
not validate actual IMU publication, VM graphics or physics.
