# openamrobot_upperbody_bringup

**Status: scaffold. Nothing is implemented here yet.**

The package contains a manifest that depends on the description, control, Gazebo and MoveIt packages, and empty `config/`
and `launch/` directories. There are no launch files.

## Intended scope

Launch composition of the base, the fixed mast and the arms, with simulation and physical-integration profiles kept
separate. No lift: OpenAMRobot 2.0 has a fixed mast. The `description` field of `package.xml` still says "base + lift +
arm" because it predates that decision (issue #7) and has not been re-worded yet.

## What exists today instead

To look at the model, use the launch file that lives in the description package, not here:

```bash
ros2 launch openamrobot_upperbody_description view.launch.py mast_id:=mast_1350
```

A mock bringup that starts the controllers is planned with the arm `ros2_control` profiles and is not implemented; its
verification check (L01) is `SKIP`.

See [`openamrobot_upperbody_description`](../openamrobot_upperbody_description/README.md).
