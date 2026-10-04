import sys
from pathlib import Path

import yaml


def fmt(v) -> str:
    return " ".join(str(x) for x in v)


ARM_PAIRS = [("base_link", "link_1"), ("link_1", "link_2"), ("link_1", "link_3"),
             ("link_1", "link_4"), ("link_2", "link_3"), ("link_3", "link_4"),
             ("link_4", "link_5"), ("link_4", "link_6"), ("link_5", "link_6")]
READY = [0, 0, 1.5707, 0, -1.5707, 0]
HUMBLE_ADAPTERS = (
    "default_planner_request_adapters/AddTimeOptimalParameterization "
    "default_planner_request_adapters/ResolveConstraintFrames "
    "default_planner_request_adapters/FixWorkspaceBounds "
    "default_planner_request_adapters/FixStartStateBounds "
    "default_planner_request_adapters/FixStartStateCollision "
    "default_planner_request_adapters/FixStartStatePathConstraints")


def gen_srdf(pkg, cell, reg):
    out = ['<?xml version="1.0"?>', '<robot name="cell">']
    for r in cell:
        n = r["name"]
        out.append(
            f'  <group name="{n}_arm"><chain base_link="{n}_base_link" tip_link="{n}_flange"/></group>')
    out.append('  <group name="all_arms">' +
               "".join(f'<group name="{r["name"]}_arm"/>' for r in cell) + '</group>')
    for r in cell:
        n = r["name"]
        for state, vals in (("home", [0] * 6), ("ready", READY)):
            js = "".join(
                f'<joint name="{n}_joint_{i+1}" value="{v}"/>' for i, v in enumerate(vals))
            out.append(
                f'  <group_state name="{state}" group="{n}_arm">{js}</group_state>')
        for a, b in ARM_PAIRS:
            out.append(
                f'  <disable_collisions link1="{n}_{a}" link2="{n}_{b}" reason="Adjacent"/>')
        if r["gripper"]:
            for a, b in reg[r["gripper"]]["srdf_pairs"]:
                pa = f"{n}_{a}" if a.startswith("link_") else f"{n}_grip_{a}"
                pb = f"{n}_{b}" if b.startswith("link_") else f"{n}_grip_{b}"
                out.append(
                    f'  <disable_collisions link1="{pa}" link2="{pb}" reason="Adjacent"/>')
    out.append('</robot>')
    (pkg / "config/cell.srdf").write_text("\n".join(out))


def gen_moveit_yaml(pkg, cell):
    groups = [f'{r["name"]}_arm' for r in cell]

    kin = {g: {"kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
               "kinematics_solver_search_resolution": 0.005,
               "kinematics_solver_timeout": 0.05} for g in groups}

    lim = {"default_velocity_scaling_factor": 0.1,
           "default_acceleration_scaling_factor": 0.1,
           "joint_limits": {}}
    for r in cell:
        for i in range(1, 7):
            lim["joint_limits"][f'{r["name"]}_joint_{i}'] = {
                "has_velocity_limits": True, "max_velocity": 3.14159,
                "has_acceleration_limits": True, "max_acceleration": 3.0}

    rrt = {"default_planner_config": "RRTConnectkConfigDefault",
           "planner_configs": ["RRTConnectkConfigDefault"]}
    ompl = {"planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": HUMBLE_ADAPTERS,
            "start_state_max_bounds_error": 0.1,
            "planner_configs": {"RRTConnectkConfigDefault": {
                "type": "geometric::RRTConnect", "range": 0.0}}}
    for g in groups + ["all_arms"]:
        ompl[g] = dict(rrt)

    ctl = {"moveit_manage_controllers": False,
           "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
           "moveit_simple_controller_manager": {
               "controller_names": [f"{g}_controller" for g in groups]}}
    for r in cell:
        n = r["name"]
        ctl["moveit_simple_controller_manager"][f"{n}_arm_controller"] = {
            "type": "FollowJointTrajectory", "action_ns": "follow_joint_trajectory",
            "default": True, "joints": [f"{n}_joint_{i}" for i in range(1, 7)]}

    for name, data in (("cell_kinematics", kin), ("cell_joint_limits", lim),
                       ("cell_ompl_planning", ompl), ("cell_moveit_controllers", ctl)):
        (pkg /
         f"config/{name}.yaml").write_text(yaml.safe_dump(data, sort_keys=False))


def main(pkg, desc_pkg_unused=None) -> None:
    pkg = Path(pkg)
    cell = yaml.safe_load((pkg / "config/cell.yaml").read_text())["robots"]
    reg = yaml.safe_load((pkg / "config/grippers.yaml").read_text())
    template: str = (pkg / "robots/cr5/robot.xml.in").read_text()

    # ---- 1. robot variants: one per gripper type actually used ----
    for g in {r["gripper"] for r in cell} | {None}:
        if g is None:
            asset, mount, fname = "", "", "robot.xml"
        else:
            c = reg[g]
            asset = (f'<asset><model name="tool" '
                     f'file="../../grippers/{g}/{g}.xml"/></asset>')
            mount = (f'<frame name="tool_mount" pos="{fmt(c["mount_pos"])}" '
                     f'euler="{fmt(c["mount_euler"])}">'
                     f'<attach model="tool" body="{c["body"]}" prefix="grip_"/></frame>')
            fname = f"robot_{g}.xml"
        (pkg / "robots/cr5" / fname).write_text(
            template.replace("<!--TOOL_ASSET-->", asset)
                    .replace("<!--TOOL_MOUNT-->", mount))

    # ---- 2. MuJoCo scene ----
    assets, frames = [], []
    for r in cell:
        variant = "robot.xml" if r["gripper"] is None else f'robot_{r["gripper"]}.xml'
        assets.append(
            f'<model name="cr5_{r["name"]}" file="../robots/cr5/{variant}"/>')
        frames.append(f'<frame name="{r["name"]}_mount" pos="{fmt(r["pos"])}">'
                      f'<attach model="cr5_{r["name"]}" body="base" prefix="{r["name"]}_"/></frame>')
    scene = f"""<mujoco model="cell">
  <compiler angle="radian" eulerseq="XYZ"/>
  <option timestep="0.002" integrator="implicitfast" gravity="0 0 -9.81"/>
  <statistic center="0 0 0.4" extent="2.5"/>
  <visual><headlight ambient="0.5 0.5 0.5" diffuse="0.8 0.8 0.8" specular="0.2 0.2 0.2"/></visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" width="300" height="300"/>
    <material name="grid" texture="grid" texrepeat="5 5" reflectance="0.1"/>
    {chr(10).join(assets)}
  </asset>
  <worldbody>
    <light name="main_light" pos="1 1 2" dir="-1 -1 -2" directional="true"/>
    <camera name="main_camera" pos="4 0 2.5" xyaxes="0 1 0 -0.5 0 0.85"/>
    <geom name="floor" type="plane" size="0 0 0.05" material="grid"/>
    {chr(10).join(frames)}
  </worldbody>
</mujoco>"""
    (pkg / "scenes/cell.xml").write_text(scene)

    # ---- 3. URDF xacro ----
    used = sorted(g for g in {r["gripper"] for r in cell} if g)
    inc = "\n".join(
        f'  <xacro:include filename="$(find eoat_description)/urdf/{g}/{g}_macro.xacro"/>'
        for g in used)
    body, joints = [], []
    for r in cell:
        n = r["name"]
        body.append(f'''  <xacro:cr5_robot prefix="{n}_"/>
  <joint name="{n}_world_to_base_joint" type="fixed">
    <parent link="world"/><child link="{n}_base_link"/>
    <origin xyz="{fmt(r["pos"])}" rpy="0 0 0"/>
  </joint>''')
        joints += [f'    <xacro:cr5_joint_interface joint_name="{n}_joint_{i}"/>'
                   for i in range(1, 7)]
        if r["gripper"]:
            g = r["gripper"]
            body.append(f'  <xacro:{g} prefix="{n}_grip_" parent="{n}_flange">'
                        f'<origin xyz="{fmt(reg[g]["urdf_origin_xyz"])}" '
                        f'rpy="{fmt(reg[g]["urdf_origin_rpy"])}"/></xacro:{g}>')
            joints += [f'    <xacro:cr5_joint_interface joint_name="{n}_grip_{j}"/>'
                       for j in reg[g]["actuated"]]
    urdf = f"""<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="cell">
  <xacro:arg name="mujoco_model" default="$(find dobot_mujoco_integration)/scenes/cell.xml"/>
  <xacro:include filename="$(find dobot_description)/urdf/cr5/cr5_macro.xacro"/>
  <xacro:include filename="$(find dobot_bringup)/urdf/cr5/cr5_controller.xacro"/>
{inc}
  <link name="world"/>
{chr(10).join(body)}
  <ros2_control name="cell_system" type="system">
    <hardware>
      <plugin>mujoco_ros2_control/MujocoSystemInterface</plugin>
      <param name="mujoco_model">$(arg mujoco_model)</param>
      <param name="auto_register_cameras">false</param>
    </hardware>
{chr(10).join(joints)}
  </ros2_control>
</robot>"""
    (pkg / "urdf").mkdir(exist_ok=True)
    (pkg / "urdf/cell.urdf.xacro").write_text(urdf)

    # ---- 4. controllers ----
    cm = {"update_rate": 500,
          "joint_state_broadcaster": {"type": "joint_state_broadcaster/JointStateBroadcaster"}}
    params = {}
    for r in cell:
        n = r["name"]
        cm[f"{n}_arm_controller"] = {
            "type": "joint_trajectory_controller/JointTrajectoryController"}
        params[f"{n}_arm_controller"] = {"ros__parameters": {
            "joints": [f"{n}_joint_{i}" for i in range(1, 7)],
            "command_interfaces": ["position"],
            "state_interfaces": ["position", "velocity"],
            "state_publish_rate": 50.0, "action_monitor_rate": 20.0,
            "allow_partial_joints_goal": False,
            "allow_nonzero_velocity_at_trajectory_end": False,
            "constraints": {"stopped_velocity_tolerance": 0.01, "goal_time": 0.0}}}
        if r["gripper"]:
            c = reg[r["gripper"]]
            cm[f"{n}_gripper_controller"] = {"type": c["controller"]}
            params[f"{n}_gripper_controller"] = {"ros__parameters": {
                "joints": [f"{n}_grip_{j}" for j in c["actuated"]],
                "interface_name": "position"}}
    out = {"controller_manager": {"ros__parameters": cm}, **params}
    (pkg / "config/cell_controllers.yaml").write_text(yaml.safe_dump(out, sort_keys=False))

    gen_srdf(pkg, cell, reg)
    gen_moveit_yaml(pkg, cell)


if __name__ == "__main__":
    main(sys.argv[1])
