# Using Jetson Nano as TurtleBot3's computer
TurtleBot3 uses the [OpenCR](https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_opencr1_0/) board as the embedded board. This board is responsible for powering the SBC, motors, as well as the sensors installed. To use Jetson Nano as the SBC, we will need to establish connection with the OpenCR Board.

## Electrical Setup and Firmware update
1. Connect Jetson Nano board to external power via power adapter. I used the board and adapter that come with the Jetbot setup, although any other power source would work as well.
2. Connect OpenCR board to power using a 12V power adapter. 
3. Update OpenCR firmware using the Shell script or Arduino method as described [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/opencr_setup/).
4. After the update, connect the USB cable between Jetson Nano and OpenCR board.

## Software Setup
1. Install TurtleBot3 ROS Melodic binaries on both Remote PC and Jetson Nano using `sudo apt install ros-melodic-turtlebot3`.
2. Follow the network configuration setup [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/#network-configuration).
3. Add the environment variable `TURTLEBOT3_MODEL=waffle_pi`, on both the remote PC and Jetson Nano by adding `export TURTLEBOT3_MODEL=waffle_pi` to the `~/.bashrc` files.

## Running ROS and receiving Odometry
1. Run `roscore` on the Remote PC.
2. On Jetson Nano, run `roslaunch turtlebot3_bringup turtlebot3_robot.launch`. The output should look like 
```
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch
... logging to /home/jetbot/.ros/log/c3625c30-dab8-11ea-aef6-185680485f76/roslaunch-jetbot-21646.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://192.168.0.113:42049/

SUMMARY
========

PARAMETERS
 * /rosdistro: melodic
 * /rosversion: 1.14.7
 * /turtlebot3_core/baud: 115200
 * /turtlebot3_core/port: /dev/ttyACM0
 * /turtlebot3_core/tf_prefix: 
 * /turtlebot3_lds/frame_id: base_scan
 * /turtlebot3_lds/port: /dev/ttyUSB0

NODES
  /
    turtlebot3_core (rosserial_python/serial_node.py)
    turtlebot3_diagnostics (turtlebot3_bringup/turtlebot3_diagnostics)
    turtlebot3_lds (hls_lfcd_lds_driver/hlds_laser_publisher)

ROS_MASTER_URI=http://192.168.0.114:11311

process[turtlebot3_core-1]: started with pid [21711]
process[turtlebot3_lds-2]: started with pid [21712]
process[turtlebot3_diagnostics-3]: started with pid [21713]
[ERROR] [1597030958.787883591]: An exception was thrown: open: No such file or directory
[turtlebot3_lds-2] process has died [pid 21712, exit code 255, cmd /opt/ros/melodic/lib/hls_lfcd_lds_driver/hlds_laser_publisher __name:=turtlebot3_lds __log:=/home/jetbot/.ros/log/c3625c30-dab8-11ea-aef6-185680485f76/turtlebot3_lds-2.log].
log file: /home/jetbot/.ros/log/c3625c30-dab8-11ea-aef6-185680485f76/turtlebot3_lds-2*.log
[INFO] [1597030960.010782]: ROS Serial Python Node
[INFO] [1597030960.056496]: Connecting to /dev/ttyACM0 at 115200 baud
[INFO] [1597030962.173868]: Requesting topics...
[INFO] [1597030962.251144]: Note: publish buffer size is 1024 bytes
[INFO] [1597030962.256490]: Setup publisher on sensor_state [turtlebot3_msgs/SensorState]
[INFO] [1597030962.274646]: Setup publisher on firmware_version [turtlebot3_msgs/VersionInfo]
[INFO] [1597030962.605411]: Setup publisher on imu [sensor_msgs/Imu]
[INFO] [1597030962.618019]: Setup publisher on cmd_vel_rc100 [geometry_msgs/Twist]
[INFO] [1597030962.750530]: Setup publisher on odom [nav_msgs/Odometry]
[INFO] [1597030962.768684]: Setup publisher on joint_states [sensor_msgs/JointState]
[INFO] [1597030962.780399]: Setup publisher on battery_state [sensor_msgs/BatteryState]
[INFO] [1597030962.795958]: Setup publisher on magnetic_field [sensor_msgs/MagneticField]
[INFO] [1597030964.717923]: Setup publisher on /tf [tf/tfMessage]
[INFO] [1597030964.737858]: Note: subscribe buffer size is 1024 bytes
[INFO] [1597030964.742870]: Setup subscriber on cmd_vel [geometry_msgs/Twist]
[INFO] [1597030964.761742]: Setup subscriber on sound [turtlebot3_msgs/Sound]
[INFO] [1597030964.780938]: Setup subscriber on motor_power [std_msgs/Bool]
[INFO] [1597030964.801349]: Setup subscriber on reset [std_msgs/Empty]
[INFO] [1597030965.316367]: Setup TF on Odometry [odom]
[INFO] [1597030965.328401]: Setup TF on IMU [imu_link]
[INFO] [1597030965.346380]: Setup TF on MagneticField [mag_link]
[INFO] [1597030965.355662]: Setup TF on JointState [base_link]
[INFO] [1597030965.370260]: --------------------------
[INFO] [1597030965.382398]: Connected to OpenCR board!
[INFO] [1597030965.393015]: This core(v1.2.3) is compatible with TB3 Waffle or Waffle Pi
[INFO] [1597030965.401989]: --------------------------
[INFO] [1597030965.413956]: Start Calibration of Gyro
[INFO] [1597030967.883384]: Calibration End

```
The `[ERROR]` is due to the fact that we are not using the laser scanner that comes with TurtleBot3.

3. On Remote PC, run `roslaunch turtlebot3_bringup turtlebot3_remote.launch`.
4. Now, we are ready to send and receive ROS messages. To check that we can receive odometry from the robot, check the output of `rostopic echo /odom` either on the Remote PC or on Jetson Nano. The output should be like
```
---
header: 
  seq: 987
  stamp: 
    secs: 1597031283
    nsecs: 652199083
  frame_id: "odom"
child_frame_id: "base_footprint"
pose: 
  pose: 
    position: 
      x: -0.147986590862
      y: -0.0244562737644
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.179800972342
      w: 0.983703017235
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
twist: 
  twist: 
    linear: 
      x: -0.11722844094
      y: 0.0
      z: 0.0
    angular: 
      x: 0.0
      y: 0.0
      z: 0.217905953526
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---

```
5. We can now try moving the Robot by publishing commands to the `/cmd_vel` topic. This can be done by launching the Teleop node using `roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch`. You should see the odometry info changing as you move the robot.