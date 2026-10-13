# V01: look at the robot

V01 has been `WARN` since A6, and it stays `WARN` until a person has looked at the robot on a screen and said what they saw.
Numeric checks are not visual ones: the 80 mm square mast section passed every test we had and was caught by someone looking.
**"Looks fine" does not clear it.** Answer each of the eight items below with what you **measured or counted**, not with how it looked:
every item names the number to read and where to read it from, and the number it should be.

## Status: WARN. The look at the mast found two defects; the lift model has not been looked at

**P14 looked at the mast (then item 1) and found two defects no automated check was examining.** The mast is gone, so that answer
describes an object that no longer exists and is not counted as an answer for the lift model: **none of the eight items below has a
recorded answer, and V01 is `WARN`.** A closed finding is not an answered checklist, and a check whose value was finding defects is
not recorded as having found nothing.

- **V01-F1, the profile was probably rotated 90 degrees. Confirmed correct by the design owner.** The mast box was 60 mm fore-aft and
  120 mm across the robot; Alex's option B has the same MISUMI HFS6-60120 cross-section 120 mm fore-aft and 60 mm across, the stiff
  direction against the arms' forward cantilever. Alex, 12 Oct: *"thanks for catching the profile orientation, that's exactly the kind
  of check we need"*. The 90 degree rotation was real. The value had been right and its axis assignment was only a prose note in the
  datum that no test asserted; the mast and `mast_section` are deleted, and the rule it pointed to is implemented (a dimension list
  declares `axis_convention` and test T12 asserts the assignment against an expansion).
- **V01-F2, the mast mass was implausible. Closed by the catalogue figure.** `mast_mass` was 3.0 kg (placeholder) for 1.2365 m of that
  profile, about 337 kg/m3 over its bounding box. The lift is a DOLD Hexalift V4 350 and its **catalogue mass is 11.7 kg** (rev19.0,
  12 Oct 2026), which lands at the top of the 7.5 to 12 kg bracket P14's estimate gave (the estimate was honest and it held). The
  rule it pointed to is implemented: a mass of ours declares a `density_band` and test T10 checks the implied density. 11.7 kg is a
  catalogue figure, not a weighing, so the datum is `provisional`.

**Item wording.** P14 found that the old item 1 listed "a thin spike" among the wrong things to see, so its own correct answer was
recorded as wrong. **Every item below states what to measure, not what it should look like**: an aesthetic judgement cannot be
falsified, a reading of 0.834 m can.

## Start the simulator and RViz

    export ROS_DOMAIN_ID=42 GZ_PARTITION=v01          # in every terminal you use below (see the note)
    source /opt/ros/jazzy/setup.bash && source ~/openamr_ws/install/setup.bash
    ros2 launch openamrobot_upperbody_gazebo sim.launch.py rviz:=true

*Note:* if `ros2 node list` already shows a `robot_state_publisher` from an earlier session (the `oa_*` helpers leave one
running), RViz would draw two models and `/tf` would carry two sources. A separate `ROS_DOMAIN_ID` and `GZ_PARTITION` keep
this run to itself.

That opens the Gazebo window and RViz beside it (fixed frame `odom`, RobotModel and TF displays). The first start takes about a
minute; wait until `ros2 control list_controllers` in another terminal shows five controllers `active`.

For items 1 to 4 the lift has to be moved, and in the simulator nothing moves it (no controller, effort 0). Use the model viewer, whose
`joint_state_publisher_gui` has a slider for `lift_joint`:

    ros2 launch openamrobot_upperbody_bringup view.launch.py base_plate:=bp000

and read frames with `ros2 run tf2_ros tf2_echo base_footprint <frame>` (translation x, y, z in metres, from the floor).

For item 8 (MoveIt) start MoveIt with RViz instead, which opens the MotionPlanning display and replaces the simulator's own RViz:

    ros2 launch openamrobot_upperbody_bringup moveit.launch.py sim:=gazebo rviz:=true physics_engine:=gz-physics-bullet-featherstone-plugin

(same `ROS_DOMAIN_ID` and `GZ_PARTITION` as above; `physics_engine` is passed so a gripper pose can close: the launch's own default is DART,
where it may not. `sim:=mock rviz:=true lift_position:=0.175` shows the same preview with no Gazebo window and the lift at its middle stop.)

## Items

Look at both windows for each item. Gazebo shows what the physics built; RViz shows what the description says. Write down the numbers.

1. **The lift column and the carriage: three readings.** `tf2_echo base_footprint lift_column_base_link` reads **x 0.060, z 0.304**
   at `bp000` (x 0.110 at `bp050`). With `lift_joint` at 0, 0.175 and 0.350 m (the slider; calibration stops `L1000`, `L1175`,
   `L1350`), `tf2_echo base_footprint openarm_left_base_link` reads **z 1.000, 1.175 and 1.350**, and its x and y do not change
   between the three (the carriage goes straight up). Record the nine numbers you read. The column has **no collision geometry** (its
   lateral side is not sourced): if you see a column drawn, say what it is.
2. **The arm profile and the two mounts: orientation and spacing.** From above in RViz (orbit to look straight down, TF display on),
   measure `arm_profile_link`'s box: **0.120 m fore-aft (x) by 0.060 m across (y)**, 0.340 m tall, its centre **0.180 m ahead** of the
   column axis. `tf2_echo lift_carriage_link openarm_left_base_link` reads **x 0.180, y +0.031, z 0.166** and the right one **y -0.031**:
   the two arm bases are **62 mm** apart. Record which side of the box is the long one (V01-F1: it must be the fore-aft side).
3. **The arms at the home pose: contact count.** At every joint zero and each of the three stops, count the pairs of arm links whose
   drawn volumes overlap (left against right, either against `arm_profile_link`, either against the base): the model's own
   collision check says **0** at all three stops and both base plates. Record your count per stop, and if it is not 0, the two link
   names and how deep. This is a reach question for A4 and Alex, not a reason to move the mounts here.
4. **The sensors: heights and directions.** `tf2_echo base_footprint head_camera_mount_link` reads **z 1.223 + lift** (1.223 at
   `L1000`, 1.573 at `L1350`) and **x 0.262** at `bp000` (0.060 column axis + 0.180 + 0.022); `head_camera_optical_frame`'s z axis
   points along the lens and is pitched **25 degrees down** at `hp25`. The two wrist cameras sit at their placeholder poses: record
   `tf2_echo openarm_left_ee_base_link left_wrist_camera_link` (y **+0.05**, z **-0.03**) and say whether that is near the hand.
5. **Driving: distance and offset.** `ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"` for 5 s of
   simulated time, then `{angular: {z: 0.5}}` for 5 s. `ros2 topic echo --once /odom` before and after: record the distance
   (about 1.0 m) and the yaw (about 2.5 rad). Then `tf2_echo base_footprint lift_carriage_link` again: it must read the same as before
   driving (the upper body goes with the base: **offset change 0.000**). Say if RViz's model and Gazebo's model are further apart
   than the width of a wheel.
   (Stop with Ctrl-C and `ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"`.)
6. **An arm trajectory: what moves.** Run the script below, or any `FollowJointTrajectory` goal on `/left_arm_controller`. Record the
   `openarm_left_joint1/2/4` values at the end (**0.3, -0.6, 1.2**, within 0.02), that `openarm_right_*` and the wheels changed by
   **less than 0.001**, and the largest single-step jump you saw in `ros2 topic echo /joint_states` at the start (it should be a
   fraction of the final value, not the final value).
7. **The gripper: both fingers.** `ros2 action send_goal /left_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.4, max_effort: 5.0}}"`
   (the right gripper's range is mirrored: use `-0.4` on `/right_gripper_controller`). Record `openarm_left_finger_joint1` and
   `openarm_left_finger_joint2` in `/joint_states` afterwards: they should be **equal within 0.01** (finger_joint2 mimics
   finger_joint1), and for the right hand opposite in sign to the left's.
8. **MoveIt: a named pose, preview and execution (A7).** In RViz's MotionPlanning panel choose Planning Group `left_arm`, open the
   Planning tab, set the Goal State to `hands_up`, press **Plan** and then **Execute**. Record the final `openarm_left_joint4`
   (**1.5**, within 0.01) of the real robot (the Gazebo window, and the Scene Robot in RViz), and that the Scene Robot's
   `openarm_left_ee_base_link` and the real one's differ by **less than 0.01 m** (`tf2_echo`). Repeat with `right_arm`, then `home` and
   `park` (`park` is a placeholder copy of `home`: its joint values should be identical to `home`'s), then the grippers
   (`left_gripper` / `right_gripper`, `closed` at **+0.7854 / -0.7854** and `open` at 0: the two hands take opposite-signed values).
   *If RViz says "Planning failed" right after the arm has been at `home`:* that should not happen any more. In Gazebo `home` sets elbow
   joint 4 exactly on its lower limit (0) and the simulator reads it a hair past it, which MoveIt refuses; `move_group` plans from a
   joint state with that noise clamped (see the MoveIt package README, "The joint-state clamp"). If you see it, say so: it is a finding.

A trajectory to run for item 6 (moves the left arm's joints 1, 2 and 4 over 4 seconds):

    ros2 action send_goal /left_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
      "{trajectory: {joint_names: [openarm_left_joint1, openarm_left_joint2, openarm_left_joint3, openarm_left_joint4, openarm_left_joint5, openarm_left_joint6, openarm_left_joint7], points: [{positions: [0.3, -0.6, 0.2, 1.2, 0.0, 0.2, 0.0], time_from_start: {sec: 4}}]}}"

## Record the answers

One line per item: the item number and the numbers you read. They go into the step's report and into the evidence for V01; the check
turns `PASS` only when all eight have an answer with the numbers, and `FAIL` if any reading is outside what the item says.

Recorded so far: **none** of the eight (P14's look was at the mast, which no longer exists; see Status above). V01 is `WARN`.
