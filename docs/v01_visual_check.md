# V01: look at the robot

V01 has been `WARN` since A6, and it stays `WARN` until a person has looked at the robot on a screen and said what they saw.
Numeric checks are not visual ones: the 80 mm square mast section passed every test we had and was caught by someone looking.
**"Looks fine" does not clear it.** Answer each item below in your own words: what you saw, and for the "wrong would look like"
case, that it did not.

## Start the simulator and RViz

    export ROS_DOMAIN_ID=42 GZ_PARTITION=v01          # in every terminal you use below (see the note)
    source /opt/ros/jazzy/setup.bash && source ~/openamr_ws/install/setup.bash
    ros2 launch openamrobot_upperbody_gazebo sim.launch.py rviz:=true

*Note:* if `ros2 node list` already shows a `robot_state_publisher` from an earlier session (the `oa_*` helpers leave one
running), RViz would draw two models and `/tf` would carry two sources. A separate `ROS_DOMAIN_ID` and `GZ_PARTITION` keep
this run to itself. The launch was checked with the Gazebo window and RViz both starting under WSLg, with no errors and five
controllers `active`; nobody has looked at either window yet, which is what this is for.

That opens the Gazebo window and RViz beside it (fixed frame `odom`, RobotModel and TF displays). The first start takes about a
minute; wait until `ros2 control list_controllers` in another terminal shows five controllers `active`.

## Items

Look at both windows for each item. Gazebo shows what the physics built; RViz shows what the description says.

1. **The mast.** A 60 x 120 mm rectangular section, taller than it is wide.
   *Wrong:* a square column, or a thin spike. Look at it from the front and from the side: the long side of the section runs along
   the robot's width (y), so it is wider from the front than from the side. That orientation is an assumption in `datums.yaml`
   (`mast_section`: "orientation unconfirmed"); say so if the geometry looks the wrong way round to you.
2. **The two arms.** Rooted on the plate at the top of the mast, symmetric about the robot's centreline, with no limb passing
   through the mast or through each other at the home pose.
   *Wrong:* an arm starting somewhere else, one arm higher than the other, an arm reaching through the mast, two arms overlapping.
3. **The plate.** Both arms on the same plate at the same height, left and right not swapped. View the robot from behind,
   looking the way it drives (+x): the left arm (`openarm_left_*`) must be on your left.
   *Wrong:* the arms at different heights, or the left arm on the right.
4. **The sensors.** The head camera above the mast top; a wrist camera near each gripper. Each is drawn as a small cube.
   *Wrong:* a large block, a camera inside the mast, a wrist camera away from the hand.
5. **Driving.** In another terminal: `ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"` for a few seconds, then
   `{angular: {z: 0.5}}`. The whole robot moves and turns and the upper body goes with it.
   *Wrong:* any part lagging behind, detaching, tilting over, or the model in RViz drifting away from the one in Gazebo.
   (Stop with Ctrl-C and `ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"`.)
6. **An arm trajectory.** Run `python3` with the script below, or any `FollowJointTrajectory` goal on `/left_arm_controller`. The arm
   moves and the base does not, and nothing jumps at the start.
   *Wrong:* the base creeping or rocking, the arm snapping to a pose in the first instant, a joint going the wrong way.
7. **The gripper (extra, the P06 mimic question).** Close a gripper:
   `ros2 action send_goal /left_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.4, max_effort: 5.0}}"`
   (the right gripper's range is mirrored: use `-0.4` on `/right_gripper_controller`). Both fingers of that hand close together,
   symmetrically.
   *Wrong:* one finger moving and the other staying still, or neither moving.

A trajectory to run for item 6 (moves the left arm's joints 1, 2 and 4 over 4 seconds):

    ros2 action send_goal /left_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
      "{trajectory: {joint_names: [openarm_left_joint1, openarm_left_joint2, openarm_left_joint3, openarm_left_joint4, openarm_left_joint5, openarm_left_joint6, openarm_left_joint7], points: [{positions: [0.3, -0.6, 0.2, 1.2, 0.0, 0.2, 0.0], time_from_start: {sec: 4}}]}}"

## Record the answers

One line per item: the item number and what you saw. They go into the step's report and into the evidence for V01; the check
turns `PASS` only when all seven have an answer that says what was seen, and `FAIL` if any item shows the wrong thing.
