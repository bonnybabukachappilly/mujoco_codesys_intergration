#!/usr/bin/env python3
"""Generate all cell models and configs from ``cell.yaml``.

Single entry point that turns the cell description into every file the
simulation and the MoveIt stack need, so that nothing is maintained twice.

Inputs (read from ``--src``, the package source directory)
----------------------------------------------------------
``config/cell.yaml``
    List of robots: ``name``, ``pos`` (x y z in metres) and ``gripper``
    (a key of ``grippers.yaml`` or ``null``).
``config/grippers.yaml``
    Tool registry: mount transform in MJCF and URDF, actuated joints,
    ros2_control controller type and SRDF collision pairs.
``robots/cr5/robot.xml.in``
    MJCF robot template with two placeholders: ``<!--TOOL_ASSET-->`` and
    ``<!--TOOL_MOUNT-->``.

Outputs (written under ``--out``, mirroring the install layout)
---------------------------------------------------------------
``robots/cr5/robot.xml``, ``robots/cr5/robot_<gripper>.xml``
    One MJCF robot variant per tool type in use, plus a tool-less one.
``scenes/cell.xml``
    MuJoCo scene that attaches every robot at its configured position.
``urdf/cell.urdf.xacro``
    Cell URDF with the ``ros2_control`` block for the MuJoCo system.
``config/cell_controllers.yaml``
    ros2_control controller manager and controller parameters.
``config/cell.srdf``
    MoveIt semantic description (groups, states, disabled collisions).
``config/cell_kinematics.yaml``, ``cell_joint_limits.yaml``,
``cell_ompl_planning.yaml``, ``cell_moveit_controllers.yaml``
    MoveIt configuration files.

Naming convention
-----------------
Robot ``r1`` owns the links ``r1_base_link`` ... ``r1_flange`` and the
joints ``r1_joint_1`` ... ``r1_joint_6``. Its tool joints are prefixed
``r1_grip_`` (for example ``r1_grip_slider_1``). The MJCF ``attach``
prefixes in the scene produce the same names, so MuJoCo, URDF and
ros2_control agree without any mapping table.

Usage
-----
    generate_cell.py --src <pkg_source_dir> --out <output_dir> [--stamp FILE]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any, Iterable

import yaml

# --------------------------------------------------------------------------
# Constants
# --------------------------------------------------------------------------

#: Body pairs of one CR5 that are disabled for collision in the SRDF.
#: Names are given without the robot prefix.
ARM_PAIRS: list[tuple[str, str]] = [
    ("base_link", "link_1"), ("link_1", "link_2"), ("link_1", "link_3"),
    ("link_1", "link_4"), ("link_2", "link_3"), ("link_3", "link_4"),
    ("link_4", "link_5"), ("link_4", "link_6"), ("link_5", "link_6"),
]

#: Joint values of the named "ready" pose (radians).
READY: list[float] = [0, 0, 1.5707, 0, -1.5707, 0]

#: Number of arm joints per robot.
ARM_JOINTS = 6

#: Planning request adapters for ROS 2 Humble (space separated, one string).
#: Jazzy replaces this mechanism, so this is the Humble-specific part.
HUMBLE_ADAPTERS = (
    "default_planner_request_adapters/AddTimeOptimalParameterization "
    "default_planner_request_adapters/ResolveConstraintFrames "
    "default_planner_request_adapters/FixWorkspaceBounds "
    "default_planner_request_adapters/FixStartStateBounds "
    "default_planner_request_adapters/FixStartStateCollision "
    "default_planner_request_adapters/FixStartStatePathConstraints"
)

#: Limits written to ``cell_joint_limits.yaml``.
#: TODO: replace with datasheet values; these are placeholders.
MAX_JOINT_VELOCITY = 3.14159
MAX_JOINT_ACCELERATION = 3.0

Cell = list[dict[str, Any]]
Registry = dict[str, dict[str, Any]]


# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------

def fmt(values: Iterable[Any]) -> str:
    """Join numbers into a space separated string for XML attributes."""
    return " ".join(str(v) for v in values)


def write(path: Path, text: str) -> None:
    """Write ``text`` to ``path`` and create missing parent directories."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)


def joint_names(robot: str) -> list[str]:
    """Return the arm joint names of ``robot`` (``r1_joint_1`` ...)."""
    return [f"{robot}_joint_{i}" for i in range(1, ARM_JOINTS + 1)]


def used_grippers(cell: Cell) -> list[str]:
    """Return the sorted gripper types that appear in the cell."""
    return sorted({r["gripper"] for r in cell if r["gripper"]})


def load_inputs(src: Path) -> tuple[Cell, Registry, str]:
    """Load and sanity-check the generator inputs.

    Returns the robot list, the gripper registry and the MJCF template.
    Raises ``ValueError`` with a readable message on bad input.
    """
    cell = yaml.safe_load((src / "config/cell.yaml").read_text())["robots"]
    reg = yaml.safe_load((src / "config/grippers.yaml").read_text()) or {}
    template = (src / "robots/cr5/robot.xml.in").read_text()

    names = [r["name"] for r in cell]
    if len(set(names)) != len(names):
        raise ValueError(f"duplicate robot names in cell.yaml: {names}")
    for r in cell:
        if len(r["pos"]) != 3:
            raise ValueError(f"robot {r['name']}: 'pos' needs 3 values")
        if r["gripper"] and r["gripper"] not in reg:
            raise ValueError(
                f"robot {r['name']}: gripper '{r['gripper']}' "
                f"is not defined in grippers.yaml")
    for marker in ("<!--TOOL_ASSET-->", "<!--TOOL_MOUNT-->"):
        if marker not in template:
            raise ValueError(f"robot.xml.in is missing the {marker} marker")
    return cell, reg, template


# --------------------------------------------------------------------------
# 1. MJCF robot variants
# --------------------------------------------------------------------------

def gen_robot_variants(out: Path, cell: Cell, reg: Registry,
                       template: str) -> None:
    """Write one MJCF robot file per tool type in use, plus a bare robot.

    The template placeholders are replaced by an ``<asset><model>`` that
    references the tool MJCF and by a ``<frame><attach>`` that mounts the
    tool body on link 6 with prefix ``grip_``.
    """
    for gripper in [None, *used_grippers(cell)]:
        if gripper is None:
            asset, mount, fname = "", "", "robot.xml"
        else:
            cfg = reg[gripper]
            asset = (f'<asset><model name="tool" '
                     f'file="../../grippers/{gripper}/{gripper}.xml"/></asset>')
            mount = (f'<frame name="tool_mount" pos="{fmt(cfg["mount_pos"])}" '
                     f'euler="{fmt(cfg["mount_euler"])}">'
                     f'<attach model="tool" body="{cfg["body"]}" '
                     f'prefix="grip_"/></frame>')
            fname = f"robot_{gripper}.xml"
        write(out / "robots/cr5" / fname,
              template.replace("<!--TOOL_ASSET-->", asset)
                      .replace("<!--TOOL_MOUNT-->", mount))


# --------------------------------------------------------------------------
# 2. MuJoCo scene
# --------------------------------------------------------------------------

def gen_scene(out: Path, cell: Cell) -> None:
    """Write ``scenes/cell.xml``: floor, light, camera and all robots."""
    assets: list[str] = []
    frames: list[str] = []
    for r in cell:
        name = r["name"]
        variant = ("robot.xml" if r["gripper"] is None
                   else f'robot_{r["gripper"]}.xml')
        assets.append(
            f'<model name="cr5_{name}" file="../robots/cr5/{variant}"/>')
        frames.append(
            f'<frame name="{name}_mount" pos="{fmt(r["pos"])}">'
            f'<attach model="cr5_{name}" body="base" prefix="{name}_"/>'
            f'</frame>')
    assets_xml = "\n    ".join(assets)
    frames_xml = "\n    ".join(frames)

    scene = f"""<mujoco model="cell">
  <compiler angle="radian" eulerseq="XYZ"/>
  <option timestep="0.002" integrator="implicitfast" gravity="0 0 -9.81"/>
  <statistic center="0 0 0.4" extent="2.5"/>
  <visual><headlight ambient="0.5 0.5 0.5" diffuse="0.8 0.8 0.8" specular="0.2 0.2 0.2"/></visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" width="300" height="300"/>
    <material name="grid" texture="grid" texrepeat="5 5" reflectance="0.1"/>
    {assets_xml}
  </asset>
  <worldbody>
    <light name="main_light" pos="1 1 2" dir="-1 -1 -2" directional="true"/>
    <camera name="main_camera" pos="4 0 2.5" xyaxes="0 1 0 -0.5 0 0.85"/>
    <geom name="floor" type="plane" size="0 0 0.05" material="grid"/>
    {frames_xml}
  </worldbody>
</mujoco>"""
    write(out / "scenes/cell.xml", scene)


# --------------------------------------------------------------------------
# 3. URDF (xacro)
# --------------------------------------------------------------------------

def gen_urdf(out: Path, cell: Cell, reg: Registry) -> None:
    """Write ``urdf/cell.urdf.xacro``.

    Contains the robot macros, the fixed world-to-base joints, the tool
    macros and the ``ros2_control`` block that selects the MuJoCo hardware
    plugin and lists every commanded joint.
    """
    includes = "\n".join(
        f'  <xacro:include filename="$(find eoat_description)'
        f'/urdf/{g}/{g}_macro.xacro"/>'
        for g in used_grippers(cell))

    body: list[str] = []
    joints: list[str] = []
    for r in cell:
        name = r["name"]
        body.append(
            f'  <xacro:cr5_robot prefix="{name}_"/>\n'
            f'  <joint name="{name}_world_to_base_joint" type="fixed">\n'
            f'    <parent link="world"/><child link="{name}_base_link"/>\n'
            f'    <origin xyz="{fmt(r["pos"])}" rpy="0 0 0"/>\n'
            f'  </joint>')
        joints += [f'    <xacro:cr5_joint_interface joint_name="{j}"/>'
                   for j in joint_names(name)]
        if r["gripper"]:
            g = r["gripper"]
            cfg = reg[g]
            body.append(
                f'  <xacro:{g} prefix="{name}_grip_" parent="{name}_flange">'
                f'<origin xyz="{fmt(cfg["urdf_origin_xyz"])}" '
                f'rpy="{fmt(cfg["urdf_origin_rpy"])}"/></xacro:{g}>')
            joints += [
                f'    <xacro:cr5_joint_interface joint_name="{name}_grip_{j}"/>'
                for j in cfg["actuated"]]

    body_xml = "\n".join(body)
    joints_xml = "\n".join(joints)
    urdf = f"""<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="cell">
  <xacro:arg name="mujoco_model" default="$(find dobot_mujoco_integration)/scenes/cell.xml"/>
  <xacro:include filename="$(find dobot_description)/urdf/cr5/cr5_macro.xacro"/>
  <xacro:include filename="$(find dobot_description)/urdf/cr5/cr5_ros2_control.xacro"/>
{includes}
  <link name="world"/>
{body_xml}
  <ros2_control name="cell_system" type="system">
    <hardware>
      <plugin>mujoco_ros2_control/MujocoSystemInterface</plugin>
      <param name="mujoco_model">$(arg mujoco_model)</param>
      <param name="auto_register_cameras">false</param>
    </hardware>
{joints_xml}
  </ros2_control>
</robot>"""
    write(out / "urdf/cell.urdf.xacro", urdf)


# --------------------------------------------------------------------------
# 4. ros2_control controllers
# --------------------------------------------------------------------------

def gen_controllers(out: Path, cell: Cell, reg: Registry) -> None:
    """Write ``config/cell_controllers.yaml``.

    One ``JointTrajectoryController`` per arm and, when a tool is fitted,
    one tool controller of the type given in ``grippers.yaml``.
    """
    manager: dict[str, Any] = {
        "update_rate": 500,
        "joint_state_broadcaster": {
            "type": "joint_state_broadcaster/JointStateBroadcaster"},
    }
    params: dict[str, Any] = {}

    for r in cell:
        name = r["name"]
        manager[f"{name}_arm_controller"] = {
            "type": "joint_trajectory_controller/JointTrajectoryController"}
        params[f"{name}_arm_controller"] = {"ros__parameters": {
            "joints": joint_names(name),
            "command_interfaces": ["position"],
            "state_interfaces": ["position", "velocity"],
            "state_publish_rate": 50.0,
            "action_monitor_rate": 20.0,
            "allow_partial_joints_goal": False,
            "allow_nonzero_velocity_at_trajectory_end": False,
            "constraints": {"stopped_velocity_tolerance": 0.01,
                            "goal_time": 0.0},
        }}
        if r["gripper"]:
            cfg = reg[r["gripper"]]
            manager[f"{name}_gripper_controller"] = {"type": cfg["controller"]}
            params[f"{name}_gripper_controller"] = {"ros__parameters": {
                "joints": [f"{name}_grip_{j}" for j in cfg["actuated"]],
                "interface_name": "position",
            }}

    config = {"controller_manager": {"ros__parameters": manager}, **params}
    write(out / "config/cell_controllers.yaml",
          yaml.safe_dump(config, sort_keys=False))


# --------------------------------------------------------------------------
# 5. MoveIt: SRDF
# --------------------------------------------------------------------------

def gen_srdf(out: Path, cell: Cell, reg: Registry) -> None:
    """Write ``config/cell.srdf``.

    Defines one planning group per arm, an ``all_arms`` group, the named
    states ``home`` and ``ready``, and the disabled collision pairs for the
    arm chain and for each fitted tool.
    """
    lines = ['<?xml version="1.0"?>', '<robot name="cell">']

    for r in cell:
        name = r["name"]
        lines.append(
            f'  <group name="{name}_arm">'
            f'<chain base_link="{name}_base_link" tip_link="{name}_flange"/>'
            f'</group>')
    members = "".join(f'<group name="{r["name"]}_arm"/>' for r in cell)
    lines.append(f'  <group name="all_arms">{members}</group>')

    for r in cell:
        name = r["name"]
        for state, values in (("home", [0] * ARM_JOINTS), ("ready", READY)):
            joints = "".join(
                f'<joint name="{name}_joint_{i + 1}" value="{v}"/>'
                for i, v in enumerate(values))
            lines.append(
                f'  <group_state name="{state}" group="{name}_arm">'
                f'{joints}</group_state>')
        for a, b in ARM_PAIRS:
            lines.append(
                f'  <disable_collisions link1="{name}_{a}" '
                f'link2="{name}_{b}" reason="Adjacent"/>')
        if r["gripper"]:
            for a, b in reg[r["gripper"]]["srdf_pairs"]:
                # arm links are "<robot>_link_x"; tool links are
                # "<robot>_grip_<link>"
                pa = f"{name}_{a}" if a.startswith(
                    "link_") else f"{name}_grip_{a}"
                pb = f"{name}_{b}" if b.startswith(
                    "link_") else f"{name}_grip_{b}"
                lines.append(
                    f'  <disable_collisions link1="{pa}" link2="{pb}" '
                    f'reason="Adjacent"/>')

    lines.append("</robot>")
    write(out / "config/cell.srdf", "\n".join(lines))


# --------------------------------------------------------------------------
# 6. MoveIt: kinematics, limits, OMPL, controller mapping
# --------------------------------------------------------------------------

def gen_moveit_yaml(out: Path, cell: Cell) -> None:
    """Write the four MoveIt YAML files (kinematics, limits, OMPL, controllers)."""
    groups = [f'{r["name"]}_arm' for r in cell]

    kinematics = {
        g: {"kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
            "kinematics_solver_search_resolution": 0.005,
            "kinematics_solver_timeout": 0.05}
        for g in groups
    }

    limits: dict[str, Any] = {
        "default_velocity_scaling_factor": 0.1,
        "default_acceleration_scaling_factor": 0.1,
        "joint_limits": {},
    }
    for r in cell:
        for joint in joint_names(r["name"]):
            limits["joint_limits"][joint] = {
                "has_velocity_limits": True,
                "max_velocity": MAX_JOINT_VELOCITY,
                "has_acceleration_limits": True,
                "max_acceleration": MAX_JOINT_ACCELERATION,
            }

    group_planning = {"default_planner_config": "RRTConnectkConfigDefault",
                      "planner_configs": ["RRTConnectkConfigDefault"]}
    ompl: dict[str, Any] = {
        "planning_plugin": "ompl_interface/OMPLPlanner",
        "request_adapters": HUMBLE_ADAPTERS,
        "start_state_max_bounds_error": 0.1,
        "planner_configs": {"RRTConnectkConfigDefault": {
            "type": "geometric::RRTConnect", "range": 0.0}},
    }
    for g in [*groups, "all_arms"]:
        ompl[g] = dict(group_planning)

    controllers: dict[str, Any] = {
        "moveit_manage_controllers": False,
        "moveit_controller_manager":
            "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_simple_controller_manager": {
            "controller_names": [f"{g}_controller" for g in groups]},
    }
    for r in cell:
        name = r["name"]
        controllers["moveit_simple_controller_manager"][
            f"{name}_arm_controller"] = {
                "type": "FollowJointTrajectory",
                "action_ns": "follow_joint_trajectory",
                "default": True,
                "joints": joint_names(name),
        }

    for fname, data in (("cell_kinematics", kinematics),
                        ("cell_joint_limits", limits),
                        ("cell_ompl_planning", ompl),
                        ("cell_moveit_controllers", controllers)):
        write(out / f"config/{fname}.yaml",
              yaml.safe_dump(data, sort_keys=False))


# --------------------------------------------------------------------------
# Entry points
# --------------------------------------------------------------------------

def main(src: Path, out: Path) -> None:
    """Run every generator step.

    ``src`` is the package source directory, ``out`` the directory that
    receives all generated files (the CMake build directory).
    """
    cell, reg, template = load_inputs(src)

    gen_robot_variants(out, cell, reg, template)
    gen_scene(out, cell)
    gen_urdf(out, cell, reg)
    gen_controllers(out, cell, reg)
    gen_srdf(out, cell, reg)
    gen_moveit_yaml(out, cell)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """Parse command line arguments."""
    ap = argparse.ArgumentParser(
        description="Generate MJCF, URDF, SRDF and controller configs "
                    "from cell.yaml.")
    ap.add_argument("--src", type=Path, required=True,
                    help="package source directory (contains config/, robots/)")
    ap.add_argument("--out", type=Path, required=True,
                    help="output directory for generated files")
    ap.add_argument("--stamp", type=Path,
                    help="file touched on success (CMake dependency stamp)")
    return ap.parse_args(argv)


if __name__ == "__main__":
    args = parse_args()
    try:
        main(args.src, args.out)
    except (ValueError, KeyError, FileNotFoundError) as exc:
        print(f"generate_cell.py: error: {exc!r}", file=sys.stderr)
        sys.exit(1)
    if args.stamp:
        args.stamp.parent.mkdir(parents=True, exist_ok=True)
        args.stamp.touch()
