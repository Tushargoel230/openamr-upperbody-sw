"""Fixed-joint forward kinematics on an expanded URDF (numpy only, no KDL, no ROS runtime).

Reusable by later steps: pass the ElementTree root of an expanded URDF and two link names.
"""
import numpy as np


def rpy_to_matrix(roll, pitch, yaw):
    """URDF convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll), fixed axes."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def pose_to_matrix(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)):
    t = np.eye(4)
    t[:3, :3] = rpy_to_matrix(*rpy)
    t[:3, 3] = xyz
    return t


def joint_origin(joint):
    """4x4 parent->child transform from a <joint> element's <origin> (identity if absent)."""
    origin = joint.find("origin")
    if origin is None:
        return np.eye(4)
    xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()]
    return pose_to_matrix(xyz, rpy)


def parent_joints(root):
    """{child link: <joint> element} for every joint of the URDF."""
    return {j.find("child").get("link"): j for j in root.findall("joint")}


def fixed_fk(root, base, target):
    """Pose of link `target` in link `base` (4x4), composed along the joint chain from `target` up to `base`.

    Every joint on the path must be `fixed`: this is a zero-pose FK, so a movable joint on the path is an error,
    not a silent zero. `base` must be an ancestor of `target` (or the same link).
    """
    by_child = parent_joints(root)
    pose = np.eye(4)
    link = target
    while link != base:
        joint = by_child.get(link)
        if joint is None:
            raise ValueError(f"{base!r} is not an ancestor of {target!r} (chain ends at {link!r})")
        if joint.get("type") != "fixed":
            raise ValueError(f"joint {joint.get('name')!r} on the path {base!r} -> {target!r} is "
                             f"{joint.get('type')!r}, not fixed")
        pose = joint_origin(joint) @ pose
        link = joint.find("parent").get("link")
    return pose


def position(pose):
    return pose[:3, 3].copy()


def zero_pose_poses(root):
    """{link name: 4x4 pose in the root link} with every joint at zero position (movable joints contribute only their
    <origin>). For flagging, not for kinematics: fixed_fk is the one that refuses movable joints."""
    by_child = parent_joints(root)
    poses = {}

    def pose(link):
        if link not in poses:
            joint = by_child.get(link)
            poses[link] = np.eye(4) if joint is None else pose(joint.find("parent").get("link")) @ joint_origin(joint)
        return poses[link]

    for element in root.findall("link"):
        pose(element.get("name"))
    return poses
