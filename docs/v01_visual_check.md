# V01: look at the robot

V01 has been `WARN` since A6, and it stays `WARN` until a person has looked at the robot on a screen and said what they saw.
Numeric checks are not visual ones: the 80 mm square mast section passed every test we had and was caught by someone looking.
**"Looks fine" does not clear it.** Answer each of the eight items below in your own words: what you saw, and for the "wrong would look like"
case, that it did not.

## Status: WARN. One item answered, two defects found

A person has now looked, at the mast (item 1), and the look found two things that no automated check was examining. V01 stays
**WARN**: items 2 to 8 have no recorded answer, and a check whose value was finding defects is not recorded as having found nothing.
The manual check found what the automated ones could not.

**What the look established.** The dimensions are right: `0.060 x 0.120 x 1.2365` m is a real MISUMI HFS6-60120 profile, 14 mm shorter
than the 1250 mm Alex specified on 28 Sep. Seen side-on (the 60 mm face) it is about 20:1 and reads as a spike; from the front (the
120 mm face) it is about 10:1. That is an accurate thin extrusion, not a sizing defect.

**What it found**

- **V01-F1, the profile is probably rotated 90 degrees.** Ours is 60 mm fore-aft and 120 mm across the robot. In Alex's option B the same
  MISUMI HFS6-60120 cross-section (his `arm_profile`, a 340 mm piece on the column top, not a full-height mast) is 120 mm fore-aft and
  60 mm across, which is the stiff direction against the arms' forward cantilever; his lift column is likewise "mounted with its 240 mm
  side fore-aft". It is the same section turned the other way, which is why this says "probably": he does not model a 1.24 m member.
  `mast_section` is `[0.060, 0.120]` with `source: plan`: the value is right, and its axis assignment was only a prose note in the datum
  ("orientation unconfirmed") that no test asserts. It also explains the spike: orbiting in RViz, the side view of this model is the
  60 mm face. The arm-mount poses at y = +/-0.060 sit on the side faces of the 120 mm width, so the box cannot be turned on its own.
- **V01-F2, the mast mass is implausible.** `mast_mass = 3.0` kg (placeholder) for 1.2365 m of that profile is about 337 kg/m3 over its
  bounding box, low by roughly 2.5 to 4 times for an aluminium extrusion. That is an estimate: the MISUMI datasheet is the source for the
  real figure. It is honestly labelled `placeholder`, so nothing lied, but nothing in the harness asks whether a placeholder is plausible.
  The modelled total of 21.96 kg is light by the difference, which makes the 93 kg F2S vehicle estimate printed beside it in T10 more
  load-bearing, not less.

**Neither is fixed here.** Both die with `mast.xacro` when option B (P-03 rev18.9 item 18) is adopted: his profile is 340 mm on the
carriage, oriented his way, and he models no masses. The rules they point to are carried forward as proposals for the design owners
(neither is in `design/VERIFICATION.md`, and nothing implements them yet):

1. every datum that is a pair or triple of dimensions declares its axis convention, and a test asserts the assignment;
2. the density implied by a mass over its bounding-box volume must fall in a sane range, or WARN, placeholders included.

**The checklist's own wording was part of it.** Item 1 below used to list "a thin spike" among the wrong things to see, and the rule
under "Record the answers" turns a wrong thing seen into `FAIL`. A 60 x 120 mm section over 1.24 m reads as a spike from the side, so
the item described its own correct answer as wrong. V01 is recorded as `WARN` (the decision of the architecture chat, P14), and item 1
is reworded so that a later reader is not given the same contradiction.

**The mast items describe the current build.** The 7 Oct hardware decisions replace the fixed mast with a lift (this branch implements the
configuration in force when it was written, and the lift migration is tracked separately), so what items 1 to 4 describe will change. Answer for
what is **on the screen now**, not for what is coming.

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

For item 8 (MoveIt) start MoveIt with RViz instead, which opens the MotionPlanning display and replaces the simulator's own RViz:

    ros2 launch openamrobot_upperbody_bringup moveit.launch.py sim:=gazebo rviz:=true physics_engine:=gz-physics-bullet-featherstone-plugin

(same `ROS_DOMAIN_ID` and `GZ_PARTITION` as above; `physics_engine` is passed so a gripper pose can close: the launch's own default is DART,
where it may not. `sim:=mock rviz:=true` shows the same preview with no Gazebo window.) The fixed frame there is `base_link`.

## Items

Look at both windows for each item. Gazebo shows what the physics built; RViz shows what the description says.

1. **The mast.** A 60 x 120 mm rectangular section, about 1.24 m tall: a thin extrusion, and it reads as a spike from the side.
   *Wrong:* a square column, or a section that is not 60 x 120. Look at it from the front and from the side: the long side of the
   section runs along the robot's width (y), so it is wider from the front than from the side. That orientation is an assumption in
   `datums.yaml` (`mast_section`: "orientation unconfirmed"), and option B has it the other way round (V01-F1); say so if the geometry
   looks the wrong way round to you. *Answered once, see Status above.*
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

8. **MoveIt: a named pose, preview and execution (A7).** In RViz's MotionPlanning panel choose Planning Group `left_arm`, open the
   Planning tab, set the Goal State to `hands_up`, press **Plan**: a purple copy of the robot animates the plan. Then press **Execute**
   and watch the real robot (the Gazebo window, and the Scene Robot in RViz). The preview matches what the robot then does: the same
   pose at the end, the same direction of motion, the same arm. Repeat with `right_arm`, then `home` and `park` (`park` is a placeholder
   copy of `home`: it should look the same as `home`), then the grippers (`left_gripper` / `right_gripper`, `closed` and `open`).
   *Wrong:* the preview showing a different pose than the robot reaches, the wrong arm moving, an arm swinging through the mast or the
   other arm in the preview, a gripper whose fingers do not close together, one hand closing while the other opens for the same named pose
   (the two hands take opposite-signed joint values, so each must close its own fingers), the robot in Gazebo and the Scene Robot in RViz
   in different poses afterwards.
   *If RViz says "Planning failed" right after the arm has been at `home`:* that should not happen any more. In Gazebo `home` sets elbow
   joint 4 exactly on its lower limit (0) and the simulator reads it a hair past it, which MoveIt refuses; `move_group` now plans from a
   joint state with that noise clamped (see the MoveIt package README, "The joint-state clamp"). If you see it, say so: it is a finding.

A trajectory to run for item 6 (moves the left arm's joints 1, 2 and 4 over 4 seconds):

    ros2 action send_goal /left_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
      "{trajectory: {joint_names: [openarm_left_joint1, openarm_left_joint2, openarm_left_joint3, openarm_left_joint4, openarm_left_joint5, openarm_left_joint6, openarm_left_joint7], points: [{positions: [0.3, -0.6, 0.2, 1.2, 0.0, 0.2, 0.0], time_from_start: {sec: 4}}]}}"

## Record the answers

One line per item: the item number and what you saw. They go into the step's report and into the evidence for V01; the check
turns `PASS` only when all eight have an answer that says what was seen, and `FAIL` if any item shows the wrong thing.

Recorded so far: item 1 (the mast), see Status above. Items 2 to 8 are open, so the check is `WARN`.
