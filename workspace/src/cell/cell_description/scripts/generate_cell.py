"""Generate cell models and configs from the cell description, one target per package.

Single sources of truth
-----------------------
==============================  ==============================================
Fact                            Lives in
==============================  ==============================================
Robots, positions, tools        ``cell_description/config/cell.yaml``,
                                ``grippers.yaml``
Kinematics, inertia, position / ``dobot_cr5_description`` URDF macro
velocity / effort limits,       (read here through ``xacro``)
damping, friction
Acceleration limits, disabled   ``dobot_cr5_description/config/cr5.yaml``
collision pairs
Servo gains, armature (sim)     ``cell_mujoco/config/cr5_sim.yaml``
==============================  ==============================================

Targets
-------
========  =====================  ===========================================
target    run by                 files written under ``--out``
========  =====================  ===========================================
urdf      cell_description       ``urdf/cell.urdf.xacro`` (links, joints,
                                tools; no ros2_control)
mujoco    cell_mujoco            ``robots/cr5/robot*.xml`` (generated from
                                the URDF), ``scenes/cell.xml``,
                                ``urdf/cell_mujoco.urdf.xacro``,
                                ``config/cell_controllers.yaml``
moveit    cell_moveit_config     ``config/cell.srdf``,
                                ``cell_kinematics.yaml``,
                                ``cell_joint_limits.yaml``,
                                ``cell_ompl_planning.yaml``,
                                ``cell_moveit_controllers.yaml``
========  =====================  ===========================================

Inputs
------
``--cell-share DIR``   installed (or source) ``cell_description`` with
                    ``config/cell.yaml`` and ``config/grippers.yaml``.
``--robot-share DIR``  installed ``dobot_cr5_description`` (``mujoco`` and
                    ``moveit``): URDF xacro and ``config/cr5.yaml``.
``--src DIR``          package source dir (``mujoco``): ``config/cr5_sim.yaml``.
``--robot-urdf FILE``  optional pre-expanded URDF; skips running ``xacro``.

Naming convention
-----------------
Robot ``r1`` owns the links ``r1_base_link`` ... ``r1_flange`` and the joints
``r1_joint_1`` ... ``r1_joint_6``. MJCF bodies carry the URDF link names, so
the ``attach`` prefix ``r1_`` gives identical names in MuJoCo and ROS. Tool
bodies and joints carry the extra prefix ``grip_`` (``r1_grip_slider_1``).

Usage
-----
    generate_cell.py {urdf,mujoco,moveit} --cell-share DIR --out DIR
                    [--robot-share DIR] [--src DIR] [--robot-urdf FILE]
                    [--stamp FILE]
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

# --------------------------------------------------------------------------
# Constants
# --------------------------------------------------------------------------

#: Package names used in ``$(find ...)`` references of generated files.
PKG_ROBOT = "dobot_cr5_description"
PKG_TOOLS = "eoat_description"
PKG_CELL = "cell_description"
PKG_MUJOCO = "cell_mujoco"

TARGETS = ("urdf", "mujoco", "moveit")

#: Root link of the robot macro and the link the tool is mounted on.
ROBOT_ROOT = "base_link"
FLANGE_LINK = "flange"

#: Joint values of the named "ready" pose (radians), one per arm joint.
READY: list[float] = [0, 0, 1.5707, 0, -1.5707, 0]

#: HUMBLE-ONLY: planning request adapters as one space separated string.
#: Jazzy replaces this mechanism. ``grep -rn HUMBLE-ONLY`` lists every
#: Humble-specific site in the workspace.
HUMBLE_ADAPTERS = (
    "default_planner_request_adapters/AddTimeOptimalParameterization "
    "default_planner_request_adapters/ResolveConstraintFrames "
    "default_planner_request_adapters/FixWorkspaceBounds "
    "default_planner_request_adapters/FixStartStateBounds "
    "default_planner_request_adapters/FixStartStateCollision "
    "default_planner_request_adapters/FixStartStatePathConstraints"
)

Cell = list[dict[str, Any]]
Registry = dict[str, dict[str, Any]]


# --------------------------------------------------------------------------
# Small helpers
# --------------------------------------------------------------------------

def fmt(values: Iterable[Any]) -> str:
    """Join numbers into a space separated string for XML attributes."""
    return " ".join(str(v) for v in values)


def write(path: Path, text: str) -> None:
    """Write ``text`` to ``path`` and create missing parent directories."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)


def warn(message: str) -> None:
    """Print a warning to stderr (visible in the build log)."""
    print(f"generate_cell.py: WARNING: {message}", file=sys.stderr)


def used_grippers(cell: Cell) -> list[str]:
    """Return the sorted gripper types that appear in the cell."""
    return sorted({r["gripper"] for r in cell if r["gripper"]})


# --------------------------------------------------------------------------
# Input loading
# --------------------------------------------------------------------------

def load_inputs(cell_share: Path) -> tuple[Cell, Registry]:
    """Load ``cell.yaml`` and ``grippers.yaml`` and sanity-check them."""
    cfg = cell_share / "config"
    cell = yaml.safe_load((cfg / "cell.yaml").read_text())["robots"]
    reg = yaml.safe_load((cfg / "grippers.yaml").read_text()) or {}

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
    return cell, reg


def load_robot_data(robot_share: Path) -> dict[str, Any]:
    """Load ``config/cr5.yaml`` (acceleration limits, collision pairs)."""
    data = yaml.safe_load((robot_share / "config/cr5.yaml").read_text())
    for key in ("acceleration_limits", "collision"):
        if key not in data:
            raise ValueError(f"cr5.yaml: missing '{key}'")
    if data["acceleration_limits"].get("source") == "placeholder":
        warn("joint acceleration limits are PLACEHOLDERS (cr5.yaml). "
             "Replace them with datasheet values.")
    return data


def load_sim(src: Path) -> dict[str, Any]:
    """Load ``config/cr5_sim.yaml`` (servo gains) from the package source."""
    sim = yaml.safe_load((src / "config/cr5_sim.yaml").read_text())
    for key in ("geoms", "servo_classes", "joints"):
        if key not in sim:
            raise ValueError(f"cr5_sim.yaml: missing '{key}'")
    return sim


# --------------------------------------------------------------------------
# URDF model (single source for kinematics, inertia and limits)
# --------------------------------------------------------------------------

@dataclass
class Link:
    """One URDF link. Numbers are kept as the original strings."""

    name: str
    mass: str = "0"
    com: str = "0 0 0"
    inertia: str = ""          # MJCF fullinertia: ixx iyy izz ixy ixz iyz
    visual: str | None = None  # mesh path relative to the robot MJCF
    collision: str | None = None


@dataclass
class Joint:
    """One revolute URDF joint with its limits and dynamics."""

    name: str
    parent: str
    child: str
    xyz: str
    rpy: str
    axis: str
    lower: str
    upper: str
    effort: str
    velocity: str
    damping: str
    friction: str


@dataclass
class Robot:
    """Parsed robot: links, arm joints (depth first) and the tool parent."""

    links: dict[str, Link]
    joints: list[Joint]
    tool_parent: str | None

    @property
    def joint_names(self) -> list[str]:
        """Arm joint names in kinematic order."""
        return [j.name for j in self.joints]

    def children(self, link: str) -> list[Joint]:
        """Joints whose parent is ``link``."""
        return [j for j in self.joints if j.parent == link]


def expand_xacro(path: Path) -> str:
    """Run ``xacro`` on ``path`` and return the plain URDF text."""
    proc = subprocess.run(["xacro", str(path)], capture_output=True, text=True)
    if proc.returncode != 0:
        raise ValueError(f"xacro failed on {path}:\n{proc.stderr}")
    return proc.stdout


def _mesh_path(el: ET.Element | None) -> str | None:
    """Map ``package://pkg/meshes/cr5/<rest>`` to ``meshes/<rest>``."""
    if el is None:
        return None
    match = re.search(r"/meshes/cr5/(.+)$", el.get("filename", ""))
    if not match:
        raise ValueError(f"unexpected mesh path: {el.get('filename')}")
    return f"meshes/{match.group(1)}"


def _check_identity(el: ET.Element | None, what: str) -> None:
    """Reject non-zero origins, which this generator does not translate."""
    if el is None:
        return
    for attr in ("xyz", "rpy"):
        if any(float(v) for v in el.get(attr, "0 0 0").split()):
            raise ValueError(f"{what}: non-zero {attr} is not supported")


def parse_urdf(text: str) -> Robot:
    """Parse a plain URDF string into a :class:`Robot`."""
    root = ET.fromstring(text)

    links: dict[str, Link] = {}
    for el in root.findall("link"):
        link = Link(el.get("name", ""))
        inertial = el.find("inertial")
        if inertial is not None:
            origin = inertial.find("origin")
            if origin is not None:
                if any(float(v) for v in origin.get("rpy", "0 0 0").split()):
                    raise ValueError(
                        f"link {link.name}: rotated inertial frame "
                        f"is not supported")
                link.com = origin.get("xyz", "0 0 0")
            link.mass = inertial.find("mass").get("value")
            i = inertial.find("inertia")
            link.inertia = " ".join(
                i.get(k) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz"))
        _check_identity(el.find("visual/origin"), f"{link.name} visual")
        _check_identity(el.find("collision/origin"), f"{link.name} collision")
        link.visual = _mesh_path(el.find("visual/geometry/mesh"))
        link.collision = _mesh_path(el.find("collision/geometry/mesh"))
        links[link.name] = link

    if ROBOT_ROOT not in links:
        raise ValueError(f"URDF has no root link '{ROBOT_ROOT}'")

    by_parent: dict[str, list[ET.Element]] = defaultdict(list)
    for el in root.findall("joint"):
        by_parent[el.find("parent").get("link")].append(el)

    joints: list[Joint] = []
    tool_parent: list[str | None] = [None]

    def walk(link: str) -> None:
        for el in by_parent.get(link, []):
            kind = el.get("type")
            child = el.find("child").get("link")
            if kind == "revolute":
                origin = el.find("origin")
                limit = el.find("limit")
                dyn = el.find("dynamics")
                joints.append(Joint(
                    name=el.get("name", ""), parent=link, child=child,
                    xyz=origin.get(
                        "xyz", "0 0 0") if origin is not None else "0 0 0",
                    rpy=origin.get(
                        "rpy", "0 0 0") if origin is not None else "0 0 0",
                    axis=el.find("axis").get("xyz"),
                    lower=limit.get("lower"), upper=limit.get("upper"),
                    effort=limit.get("effort"), velocity=limit.get("velocity"),
                    damping=dyn.get(
                        "damping", "0") if dyn is not None else "0",
                    friction=dyn.get(
                        "friction", "0") if dyn is not None else "0",
                ))
                walk(child)
            elif kind == "fixed" and child == FLANGE_LINK:
                tool_parent[0] = link
            else:
                raise ValueError(
                    f"unsupported joint '{el.get('name')}' of type '{kind}'")

    walk(ROBOT_ROOT)
    return Robot(links, joints, tool_parent[0])


def load_robot(robot_share: Path, urdf_override: Path | None) -> Robot:
    """Expand the CR5 xacro (or read the override) and parse it."""
    if urdf_override is not None:
        text = urdf_override.read_text()
    else:
        text = expand_xacro(robot_share / "urdf/cr5/cr5.urdf.xacro")
    return parse_urdf(text)


# --------------------------------------------------------------------------
# Collision pairs (single list, emitted to SRDF and MJCF)
# --------------------------------------------------------------------------

def arm_pairs(data: dict[str, Any]) -> list[tuple[str, str, str]]:
    """Return ``(link1, link2, reason)`` for the arm from ``cr5.yaml``."""
    return [(p["link1"], p["link2"], p.get("reason", "Adjacent"))
            for p in data["collision"]["disabled_pairs"]]


def arm_link_names(data: dict[str, Any]) -> set[str]:
    """Return the arm link names that occur in the pair list."""
    return {n for a, b, _ in arm_pairs(data) for n in (a, b)}


# --------------------------------------------------------------------------
# Target urdf  (cell_description)
# --------------------------------------------------------------------------

def gen_cell_urdf(out: Path, cell: Cell, reg: Registry) -> None:
    """Write ``urdf/cell.urdf.xacro``: the robots and tools, no hardware."""
    lines = [
        '<?xml version="1.0"?>',
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="cell">',
        f'  <xacro:include filename="$(find {PKG_ROBOT})'
        f'/urdf/cr5/cr5_macro.xacro"/>',
    ]
    for g in used_grippers(cell):
        lines.append(
            f'  <xacro:include filename="$(find {PKG_TOOLS})'
            f'/urdf/{g}/{g}_macro.xacro"/>')
    lines.append('  <link name="world"/>')

    for r in cell:
        name = r["name"]
        lines += [
            f'  <xacro:cr5_robot prefix="{name}_"/>',
            f'  <joint name="{name}_world_to_base_joint" type="fixed">',
            f'    <parent link="world"/><child link="{name}_base_link"/>',
            f'    <origin xyz="{fmt(r["pos"])}" rpy="0 0 0"/>',
            '  </joint>',
        ]
        if r["gripper"]:
            g = r["gripper"]
            cfg = reg[g]
            lines.append(
                f'  <xacro:{g} prefix="{name}_grip_" parent="{name}_flange">'
                f'<origin xyz="{fmt(cfg["urdf_origin_xyz"])}" '
                f'rpy="{fmt(cfg["urdf_origin_rpy"])}"/></xacro:{g}>')

    lines.append("</robot>")
    write(out / "urdf/cell.urdf.xacro", "\n".join(lines))


# --------------------------------------------------------------------------
# Target mujoco  (cell_mujoco)
# --------------------------------------------------------------------------

def servo_params(sim: dict[str, Any], joint: str) -> dict[str, Any]:
    """Return armature/kp/kv for ``joint``: class values plus overrides."""
    entry = sim["joints"].get(joint)
    if entry is None:
        raise ValueError(f"cr5_sim.yaml: no entry for joint '{joint}'")
    params = dict(sim["servo_classes"][entry["class"]])
    params.update({k: v for k, v in entry.items() if k != "class"})
    for key in ("armature", "kp", "kv"):
        if key not in params:
            raise ValueError(f"cr5_sim.yaml: joint '{joint}' lacks '{key}'")
    return params


def _body_lines(robot: Robot, link_name: str, joint: Joint | None,
                sim: dict[str, Any], mount: str, depth: int) -> list[str]:
    """Return the MJCF lines of ``link_name`` and everything below it."""
    pad = "  " * depth
    link = robot.links[link_name]
    attrs = f'name="{link_name}"'
    if joint is not None:
        attrs += f' pos="{joint.xyz}" euler="{joint.rpy}"'
    lines = [f"{pad}<body {attrs}>"]

    if joint is not None:
        p = servo_params(sim, joint.name)
        lines.append(
            f'{pad}  <joint name="{joint.name}" type="hinge" '
            f'axis="{joint.axis}" range="{joint.lower} {joint.upper}" '
            f'armature="{p["armature"]}" damping="{joint.damping}" '
            f'frictionloss="{joint.friction}"/>')
    if link.inertia:
        lines.append(
            f'{pad}  <inertial pos="{link.com}" mass="{link.mass}" '
            f'fullinertia="{link.inertia}"/>')
    if link.visual:
        lines.append(
            f'{pad}  <geom class="cr5_visual" mesh="{link_name}_visual"/>')
    if link.collision:
        lines.append(
            f'{pad}  <geom class="cr5_collision" mesh="{link_name}_collision"/>')

    for child in robot.children(link_name):
        lines += _body_lines(robot, child.child, child, sim, mount, depth + 1)
    if mount and link_name == robot.tool_parent:
        lines.append(f"{pad}  {mount}")
    lines.append(f"{pad}</body>")
    return lines


def robot_mjcf(robot: Robot, sim: dict[str, Any], data: dict[str, Any],
               gripper: str | None, reg: Registry) -> str:
    """Build the MJCF text of one robot variant (bare or with a tool)."""
    arm_links = arm_link_names(data)
    unknown = arm_links - set(robot.links)
    if unknown:
        raise ValueError(
            f"cr5.yaml: pair links not in URDF: {sorted(unknown)}")

    asset: list[str] = []
    for link in robot.links.values():
        if link.visual:
            asset.append(
                f'    <mesh name="{link.name}_visual" file="{link.visual}"/>')
        if link.collision:
            asset.append(
                f'    <mesh name="{link.name}_collision" file="{link.collision}"/>')

    mount = ""
    excludes = [f'    <exclude body1="{a}" body2="{b}"/>'
                for a, b, _ in arm_pairs(data)]
    if gripper:
        cfg = reg[gripper]
        if robot.tool_parent is None:
            raise ValueError("URDF has no flange; cannot mount a tool")
        asset.append(f'    <model name="tool" '
                     f'file="../../grippers/{gripper}/{gripper}.xml"/>')
        mount = (f'<frame name="tool_mount" pos="{fmt(cfg["mount_pos"])}" '
                 f'euler="{fmt(cfg["mount_euler"])}">'
                 f'<attach model="tool" body="{cfg["body"]}" '
                 f'prefix="grip_"/></frame>')
        # Tool-internal pairs stay in the tool MJCF; only arm<->tool here.
        for a, b in cfg["disabled_collisions"]:
            if (a in arm_links) != (b in arm_links):
                arm, tool = (a, b) if a in arm_links else (b, a)
                excludes.append(
                    f'    <exclude body1="{arm}" body2="grip_{tool}"/>')

    body = _body_lines(robot, ROBOT_ROOT, None, sim, mount, 2)

    actuators = []
    for j in robot.joints:
        p = servo_params(sim, j.name)
        actuators.append(
            f'    <position name="{j.name}" joint="{j.name}" '
            f'ctrlrange="{j.lower} {j.upper}" kp="{p["kp"]}" kv="{p["kv"]}" '
            f'forcerange="-{j.effort} {j.effort}"/>')

    rgba = fmt(sim["geoms"]["visual_rgba"])
    nl = "\n"
    return f"""<mujoco model="cr5">
  <compiler angle="radian" eulerseq="XYZ"/>
  <default>
    <default class="cr5_visual">
      <geom type="mesh" contype="0" conaffinity="0" group="1" rgba="{rgba}"/>
    </default>
    <default class="cr5_collision">
      <geom type="mesh" group="3"/>
    </default>
  </default>
  <asset>
{nl.join(asset)}
  </asset>
  <worldbody>
{nl.join(body)}
  </worldbody>
  <contact>
{nl.join(excludes)}
  </contact>
  <actuator>
{nl.join(actuators)}
  </actuator>
</mujoco>
"""


def gen_robot_variants(out: Path, cell: Cell, reg: Registry, robot: Robot,
                       sim: dict[str, Any], data: dict[str, Any]) -> None:
    """Write one MJCF robot file per tool type in use, plus a bare robot."""
    for gripper in [None, *used_grippers(cell)]:
        fname = "robot.xml" if gripper is None else f"robot_{gripper}.xml"
        write(out / "robots/cr5" / fname,
              robot_mjcf(robot, sim, data, gripper, reg))


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
            f'<attach model="cr5_{name}" body="{ROBOT_ROOT}" prefix="{name}_"/>'
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


def gen_mujoco_urdf(out: Path, cell: Cell, reg: Registry, robot: Robot) -> None:
    """Write ``urdf/cell_mujoco.urdf.xacro`` (ros2_control + MuJoCo plugin)."""
    joints: list[str] = []
    for r in cell:
        name = r["name"]
        joints += [f'    <xacro:cr5_joint_interface joint_name="{name}_{j}"/>'
                   for j in robot.joint_names]
        if r["gripper"]:
            joints += [
                f'    <xacro:cr5_joint_interface '
                f'joint_name="{name}_grip_{j}"/>'
                for j in reg[r["gripper"]]["actuated"]]
    joints_xml = "\n".join(joints)

    urdf = f"""<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="cell">
  <xacro:arg name="mujoco_model" default="$(find {PKG_MUJOCO})/scenes/cell.xml"/>
  <xacro:include filename="$(find {PKG_CELL})/urdf/cell.urdf.xacro"/>
  <xacro:include filename="$(find {PKG_ROBOT})/urdf/cr5/cr5_ros2_control.xacro"/>
  <ros2_control name="cell_system" type="system">
    <hardware>
      <plugin>mujoco_ros2_control/MujocoSystemInterface</plugin>
      <param name="mujoco_model">$(arg mujoco_model)</param>
      <param name="auto_register_cameras">false</param>
    </hardware>
{joints_xml}
  </ros2_control>
</robot>"""
    write(out / "urdf/cell_mujoco.urdf.xacro", urdf)


def gen_controllers(out: Path, cell: Cell, reg: Registry, robot: Robot) -> None:
    """Write ``config/cell_controllers.yaml`` (arm and tool controllers)."""
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
            "joints": [f"{name}_{j}" for j in robot.joint_names],
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
# Target moveit  (cell_moveit_config)
# --------------------------------------------------------------------------

def gen_srdf(out: Path, cell: Cell, reg: Registry, robot: Robot,
             data: dict[str, Any]) -> None:
    """Write ``config/cell.srdf`` from the single collision list."""
    if len(READY) != len(robot.joints):
        raise ValueError(
            f"READY has {len(READY)} values, robot has {len(robot.joints)} joints")
    arm_links = arm_link_names(data)
    lines = ['<?xml version="1.0"?>', '<robot name="cell">']

    for r in cell:
        name = r["name"]
        lines.append(
            f'  <group name="{name}_arm">'
            f'<chain base_link="{name}_{ROBOT_ROOT}" tip_link="{name}_{FLANGE_LINK}"/>'
            f'</group>')
    members = "".join(f'<group name="{r["name"]}_arm"/>' for r in cell)
    lines.append(f'  <group name="all_arms">{members}</group>')

    for r in cell:
        name = r["name"]
        for state, values in (("home", [0] * len(robot.joints)), ("ready", READY)):
            joints = "".join(
                f'<joint name="{name}_{j}" value="{v}"/>'
                for j, v in zip(robot.joint_names, values))
            lines.append(
                f'  <group_state name="{state}" group="{name}_arm">'
                f'{joints}</group_state>')
        for a, b, reason in arm_pairs(data):
            lines.append(
                f'  <disable_collisions link1="{name}_{a}" '
                f'link2="{name}_{b}" reason="{reason}"/>')
        if r["gripper"]:
            for a, b in reg[r["gripper"]]["disabled_collisions"]:
                pa = f"{name}_{a}" if a in arm_links else f"{name}_grip_{a}"
                pb = f"{name}_{b}" if b in arm_links else f"{name}_grip_{b}"
                lines.append(
                    f'  <disable_collisions link1="{pa}" link2="{pb}" '
                    f'reason="Adjacent"/>')

    lines.append("</robot>")
    write(out / "config/cell.srdf", "\n".join(lines))


def gen_moveit_yaml(out: Path, cell: Cell, robot: Robot,
                    data: dict[str, Any]) -> None:
    """Write the four MoveIt YAML files (kinematics, limits, OMPL, controllers)."""
    groups = [f'{r["name"]}_arm' for r in cell]

    kinematics = {
        g: {"kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
            "kinematics_solver_search_resolution": 0.005,
            "kinematics_solver_timeout": 0.05}
        for g in groups
    }

    accel = data["acceleration_limits"]["values"]
    limits: dict[str, Any] = {
        "default_velocity_scaling_factor": 0.1,
        "default_acceleration_scaling_factor": 0.1,
        "joint_limits": {},
    }
    for r in cell:
        for j in robot.joints:
            if j.name not in accel:
                raise ValueError(
                    f"cr5.yaml: no acceleration limit for {j.name}")
            limits["joint_limits"][f'{r["name"]}_{j.name}'] = {
                "has_velocity_limits": True,
                "max_velocity": float(j.velocity),
                "has_acceleration_limits": True,
                "max_acceleration": float(accel[j.name]),
            }

    group_planning = {"default_planner_config": "RRTConnectkConfigDefault",
                      "planner_configs": ["RRTConnectkConfigDefault"]}
    ompl: dict[str, Any] = {
        "planning_plugin": "ompl_interface/OMPLPlanner",
        "request_adapters": HUMBLE_ADAPTERS,  # HUMBLE-ONLY
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
                "joints": [f"{name}_{j}" for j in robot.joint_names],
        }

    for fname, content in (("cell_kinematics", kinematics),
                           ("cell_joint_limits", limits),
                           ("cell_ompl_planning", ompl),
                           ("cell_moveit_controllers", controllers)):
        write(out / f"config/{fname}.yaml",
              yaml.safe_dump(content, sort_keys=False))


# --------------------------------------------------------------------------
# Entry points
# --------------------------------------------------------------------------

def main(
        target: str, cell_share: Path, out: Path, src: Path | None = None,
        robot_share: Path | None = None, robot_urdf: Path | None = None) -> None:
    """Run the generator steps that belong to ``target``."""
    cell, reg = load_inputs(cell_share)

    if target == "urdf":
        gen_cell_urdf(out, cell, reg)
        return

    if robot_share is None:
        raise ValueError(f"target '{target}' requires --robot-share")
    data = load_robot_data(robot_share)
    robot = load_robot(robot_share, robot_urdf)

    if target == "mujoco":
        if src is None:
            raise ValueError("target 'mujoco' requires --src")
        sim = load_sim(src)
        gen_robot_variants(out, cell, reg, robot, sim, data)
        gen_scene(out, cell)
        gen_mujoco_urdf(out, cell, reg, robot)
        gen_controllers(out, cell, reg, robot)
    elif target == "moveit":
        gen_srdf(out, cell, reg, robot, data)
        gen_moveit_yaml(out, cell, robot, data)
    else:
        raise ValueError(f"unknown target '{target}'")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """Parse command line arguments."""
    ap = argparse.ArgumentParser(
        description="Generate cell models and configs from the cell description.")
    ap.add_argument("target", choices=TARGETS,
                    help="which package's files to generate")
    ap.add_argument("--cell-share", type=Path, required=True,
                    help="dir containing config/cell.yaml and grippers.yaml")
    ap.add_argument("--robot-share", type=Path,
                    help="dobot_cr5_description share dir (mujoco, moveit)")
    ap.add_argument("--src", type=Path,
                    help="package source dir (required for target 'mujoco')")
    ap.add_argument("--robot-urdf", type=Path,
                    help="pre-expanded URDF; skips running xacro")
    ap.add_argument("--out", type=Path, required=True,
                    help="output directory for generated files")
    ap.add_argument("--stamp", type=Path,
                    help="file touched on success (CMake dependency stamp)")
    args = ap.parse_args(argv)
    if args.target in ("mujoco", "moveit") and args.robot_share is None:
        ap.error(f"target '{args.target}' requires --robot-share")
    if args.target == "mujoco" and args.src is None:
        ap.error("target 'mujoco' requires --src")
    return args


if __name__ == "__main__":
    args = parse_args()
    try:
        main(
            args.target, args.cell_share, args.out, args.src,
            args.robot_share, args.robot_urdf)
    except (ValueError, KeyError, FileNotFoundError, yaml.YAMLError,
            ET.ParseError) as exc:
        print(f"generate_cell.py: error: {exc!r}", file=sys.stderr)
        sys.exit(1)
    if args.stamp:
        args.stamp.parent.mkdir(parents=True, exist_ok=True)
        args.stamp.touch()
