"""Tests for scripts/generate_cell.py.

Most tests need no ROS: they run the generator on a pre-expanded URDF
fixture (``data/cr5_expanded.urdf``) together with the real YAML files of the
repository. One test compares the fixture with the real xacro and is skipped
when ROS is not available.
"""

import math
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml

import generate_cell as gc

TOOL_ARM_PAIRS = {('link_5', 'grip_gripper_base'),
                  ('link_6', 'grip_gripper_base')}
TOOL_INTERNAL_PAIRS = 3     # gripper_base/finger_1, /finger_2, finger_1/finger_2


def bodies(root):
    """Map body name to element for every body below ``root``."""
    return {b.get('name'): b for b in root.iter('body')}


def exclude_pairs(root):
    """Return the set of ``(body1, body2)`` contact exclusions."""
    return {(e.get('body1'), e.get('body2'))
            for e in root.findall('contact/exclude')}


# --------------------------------------------------------------------------
# URDF parsing: the URDF is the source of truth, so the parser must be strict
# --------------------------------------------------------------------------

def test_joint_order_and_tool_parent(robot):
    assert robot.joint_names == [f'joint_{i}' for i in range(1, 7)]
    assert robot.tool_parent == 'link_6'


@pytest.mark.parametrize('link, kind, expected', [
    ('base_link', 'visual', 'meshes/visual/base.STL'),
    ('link_3', 'collision', 'meshes/collision/link_3.STL'),
])
def test_mesh_paths_are_relative_to_the_robot_mjcf(robot, link, kind, expected):
    assert getattr(robot.links[link], kind) == expected


def test_limits_and_dynamics_are_kept_verbatim(robot):
    j4 = robot.joints[3]
    assert (j4.effort, j4.damping, j4.friction) == ('50.0', '1.0', '0.2')


@pytest.mark.parametrize('old, new', [
    ('name="joint_3" type="revolute"', 'name="joint_3" type="prismatic"'),
    ('xyz="1.732462e-07 -0.003254572 -0.007633470" rpy="0 0 0"',
     'xyz="1.732462e-07 -0.003254572 -0.007633470" rpy="0 0 0.5"'),
], ids=['prismatic-joint', 'rotated-inertial-frame'])
def test_rejects_unsupported_urdf(urdf_text, old, new):
    assert old in urdf_text
    with pytest.raises(ValueError):
        gc.parse_urdf(urdf_text.replace(old, new))


# --------------------------------------------------------------------------
# MJCF: a faithful copy of the URDF data
# --------------------------------------------------------------------------

def test_body_tree_matches_urdf(bare_robot, robot):
    tree = bodies(bare_robot)
    assert list(tree) == ['base_link', 'link_1', 'link_2', 'link_3',
                          'link_4', 'link_5', 'link_6']
    for joint in robot.joints:
        body = tree[joint.child]
        assert body.get('pos') == joint.xyz
        assert body.get('euler') == joint.rpy
        assert body.find('joint').get('range') == f'{joint.lower} {joint.upper}'


def test_inertials_are_copied_verbatim(bare_robot, robot):
    tree = bodies(bare_robot)
    for name, link in robot.links.items():
        if not link.inertia:
            continue
        inertial = tree[name].find('inertial')
        assert inertial.get('mass') == link.mass
        assert inertial.get('pos') == link.com
        assert inertial.get('fullinertia') == link.inertia


def test_fullinertia_order_is_independent_of_the_parser(bare_robot):
    # Literal values from the original hand-written robot.xml.in.
    # MJCF fullinertia order is: ixx iyy izz ixy ixz iyz.
    link1 = bodies(bare_robot)['link_1'].find('inertial')
    assert link1.get('mass') == '1.613090768'
    assert link1.get('fullinertia') == (
        '0.005467836 0.005250346 0.003409568 '
        '1.551051e-09 -7.416569e-08 -9.788468e-05')


def test_damping_and_friction_come_from_the_urdf(bare_robot, robot):
    tree = bodies(bare_robot)
    for joint in robot.joints:
        el = tree[joint.child].find('joint')
        assert el.get('damping') == joint.damping
        assert el.get('frictionloss') == joint.friction


@pytest.mark.parametrize('joint, attr, expected', [
    ('joint_1', 'kp', '8100'),
    ('joint_2', 'kp', '8600'),                 # documented override
    ('joint_4', 'kp', '500'),
    ('joint_4', 'forcerange', '-50.0 50.0'),   # from the URDF effort
    ('joint_3', 'ctrlrange', '-2.792 2.792'),  # from the URDF limits
])
def test_actuator_values(bare_robot, joint, attr, expected):
    actuators = {a.get('joint'): a for a in bare_robot.iter('position')}
    assert actuators[joint].get(attr) == expected


def test_arm_excludes_come_from_cr5_yaml(bare_robot, cr5_data):
    expected = {(a, b) for a, b, _ in gc.arm_pairs(cr5_data)}
    assert exclude_pairs(bare_robot) == expected
    assert ('link_1', 'link_3') in expected    # non-adjacent, listed on purpose


def test_tool_variant_adds_only_arm_to_tool_excludes(tool_robot, cr5_data):
    arm = {(a, b) for a, b, _ in gc.arm_pairs(cr5_data)}
    # tool-internal pairs stay in the tool MJCF, so they must not repeat here
    assert exclude_pairs(tool_robot) == arm | TOOL_ARM_PAIRS


def test_tool_is_attached_to_the_flange_parent(tool_robot, bare_robot):
    attach = next(bodies(tool_robot)['link_6'].iter('attach'))
    assert (attach.get('body'), attach.get('prefix')) == ('gripper_base', 'grip_')
    assert list(bare_robot.iter('attach')) == []


# --------------------------------------------------------------------------
# Scene, ros2_control, cell URDF
# --------------------------------------------------------------------------

def test_scene_attaches_every_robot(mujoco_out):
    scene = ET.parse(mujoco_out / 'scenes/cell.xml').getroot()
    attaches = [(a.get('model'), a.get('body'), a.get('prefix'))
                for a in scene.iter('attach')]
    assert attaches == [('cr5_r1', 'base_link', 'r1_'),
                        ('cr5_r2', 'base_link', 'r2_')]
    files = {m.get('name'): m.get('file') for m in scene.iter('model')}
    assert files['cr5_r1'] == '../robots/cr5/robot_pgc_50_35.xml'
    assert files['cr5_r2'] == '../robots/cr5/robot.xml'


def test_mujoco_urdf_lists_every_commanded_joint(mujoco_out):
    text = (mujoco_out / 'urdf/cell_mujoco.urdf.xacro').read_text()
    assert text.count('<xacro:cr5_joint_interface') == 13    # 6 + 6 + slider
    assert 'joint_name="r1_grip_slider_1"' in text
    assert 'mujoco_ros2_control/MujocoSystemInterface' in text


def test_controllers(mujoco_out):
    cfg = yaml.safe_load((mujoco_out / 'config/cell_controllers.yaml').read_text())
    assert cfg['r1_arm_controller']['ros__parameters']['joints'] == [
        f'r1_joint_{i}' for i in range(1, 7)]
    assert cfg['r1_gripper_controller']['ros__parameters']['joints'] == [
        'r1_grip_slider_1']
    assert 'r2_gripper_controller' not in cfg


def test_cell_urdf_has_no_hardware(generate):
    text = (generate('urdf') / 'urdf/cell.urdf.xacro').read_text()
    assert 'ros2_control' not in text
    assert text.count('<xacro:cr5_robot') == 2
    assert text.count('<xacro:pgc_50_35') == 1
    ET.fromstring(text)                                       # well-formed


# --------------------------------------------------------------------------
# MoveIt: derived from the same data as the MJCF
# --------------------------------------------------------------------------

def test_srdf_pairs_match_the_single_list(moveit_out, cr5_data):
    root = ET.parse(moveit_out / 'config/cell.srdf').getroot()
    by_pair = {(p.get('link1'), p.get('link2')): p.get('reason')
               for p in root.findall('disable_collisions')}
    for robot_name in ('r1', 'r2'):
        for a, b, reason in gc.arm_pairs(cr5_data):
            assert by_pair[(f'{robot_name}_{a}', f'{robot_name}_{b}')] == reason
    assert ('r1_link_5', 'r1_grip_gripper_base') in by_pair
    assert not any(k[0].startswith('r2_grip') for k in by_pair)
    expected = 2 * len(gc.arm_pairs(cr5_data)) + len(TOOL_ARM_PAIRS) + TOOL_INTERNAL_PAIRS
    assert len(by_pair) == expected


def test_srdf_groups_and_states(moveit_out):
    root = ET.parse(moveit_out / 'config/cell.srdf').getroot()
    assert {g.get('name') for g in root.findall('group')} == {
        'r1_arm', 'r2_arm', 'all_arms'}
    states = {(s.get('name'), s.get('group')) for s in root.findall('group_state')}
    assert ('ready', 'r2_arm') in states


def test_limits_velocity_from_urdf_acceleration_from_cr5_yaml(moveit_out, cr5_data):
    limits = yaml.safe_load(
        (moveit_out / 'config/cell_joint_limits.yaml').read_text())['joint_limits']
    assert len(limits) == 12
    for name, lim in limits.items():
        joint = name.split('_', 1)[1]
        assert lim['max_velocity'] == 3.14159
        assert lim['max_acceleration'] == float(
            cr5_data['acceleration_limits']['values'][joint])


def test_placeholder_warning_follows_the_source_field(generate, cr5_data, capsys):
    generate('moveit')
    is_placeholder = cr5_data['acceleration_limits']['source'] == 'placeholder'
    assert ('PLACEHOLDER' in capsys.readouterr().err) == is_placeholder


def test_moveit_controller_mapping(moveit_out):
    cfg = yaml.safe_load(
        (moveit_out / 'config/cell_moveit_controllers.yaml').read_text())
    mgr = cfg['moveit_simple_controller_manager']
    assert mgr['controller_names'] == ['r1_arm_controller', 'r2_arm_controller']
    assert mgr['r2_arm_controller']['joints'][0] == 'r2_joint_1'


# --------------------------------------------------------------------------
# The REAL cell.yaml: expectations are derived from it, never hard-coded, so
# adding a robot or a tool must not break these tests.
# --------------------------------------------------------------------------

def test_real_cell_urdf(generate, paths, real_cell):
    cell, _ = real_cell
    out = generate('urdf', cell_share=paths.real_cell_share)
    text = (out / 'urdf/cell.urdf.xacro').read_text()
    assert text.count('<xacro:cr5_robot') == len(cell)
    ET.fromstring(text)


def test_real_cell_mujoco(generate, paths, real_cell, robot):
    cell, reg = real_cell
    out = generate('mujoco', cell_share=paths.real_cell_share)

    tool_joints = sum(len(reg[r['gripper']]['actuated'])
                      for r in cell if r['gripper'])
    urdf = (out / 'urdf/cell_mujoco.urdf.xacro').read_text()
    assert urdf.count('<xacro:cr5_joint_interface') == (
        len(cell) * len(robot.joints) + tool_joints)

    scene = ET.parse(out / 'scenes/cell.xml').getroot()
    assert [a.get('prefix') for a in scene.iter('attach')] == [
        f"{r['name']}_" for r in cell]

    cfg = yaml.safe_load((out / 'config/cell_controllers.yaml').read_text())
    for r in cell:
        assert f"{r['name']}_arm_controller" in cfg
        assert (f"{r['name']}_gripper_controller" in cfg) == bool(r['gripper'])

    for xml_file in (out / 'robots/cr5').glob('*.xml'):
        ET.parse(xml_file)                                   # well-formed


def test_real_cell_moveit(generate, paths, real_cell, robot, cr5_data):
    cell, reg = real_cell
    out = generate('moveit', cell_share=paths.real_cell_share)

    srdf = ET.parse(out / 'config/cell.srdf').getroot()
    assert {g.get('name') for g in srdf.findall('group')} == (
        {f"{r['name']}_arm" for r in cell} | {'all_arms'})
    expected_pairs = (
        len(cell) * len(gc.arm_pairs(cr5_data))
        + sum(len(reg[r['gripper']]['disabled_collisions'])
              for r in cell if r['gripper']))
    assert len(srdf.findall('disable_collisions')) == expected_pairs

    limits = yaml.safe_load(
        (out / 'config/cell_joint_limits.yaml').read_text())['joint_limits']
    assert len(limits) == len(cell) * len(robot.joints)


# --------------------------------------------------------------------------
# Validation: bad input must fail with a readable error
# --------------------------------------------------------------------------

@pytest.mark.parametrize('cell_yaml, message', [
    ('robots:\n  - {name: r1, pos: [0,0,0], gripper: null}\n'
     '  - {name: r1, pos: [1,0,0], gripper: null}\n', 'duplicate'),
    ('robots:\n  - {name: r1, pos: [0,0,0], gripper: nope}\n', 'nope'),
    ('robots:\n  - {name: r1, pos: [0,0], gripper: null}\n', 'pos'),
], ids=['duplicate-name', 'unknown-gripper', 'bad-position'])
def test_bad_cell_yaml(tmp_path, paths, cell_yaml, message):
    share = tmp_path / 'share'
    shutil.copytree(paths.cell_share / 'config', share / 'config')
    (share / 'config/cell.yaml').write_text(cell_yaml)
    with pytest.raises(ValueError, match=message):
        gc.load_inputs(share)


def test_missing_servo_entry(tmp_path, paths, generate):
    sim = yaml.safe_load((paths.mujoco_src / 'config/cr5_sim.yaml').read_text())
    del sim['joints']['joint_4']
    (tmp_path / 'src/config').mkdir(parents=True)
    (tmp_path / 'src/config/cr5_sim.yaml').write_text(yaml.safe_dump(sim))
    with pytest.raises(ValueError, match='joint_4'):
        generate('mujoco', src=tmp_path / 'src')


def test_pair_link_not_in_urdf(tmp_path, paths, cr5_data, generate):
    cr5_data['collision']['disabled_pairs'].append(
        {'link1': 'link_1', 'link2': 'link_99', 'reason': 'Typo'})
    (tmp_path / 'robot/config').mkdir(parents=True)
    (tmp_path / 'robot/config/cr5.yaml').write_text(yaml.safe_dump(cr5_data))
    with pytest.raises(ValueError, match='link_99'):
        generate('mujoco', robot_share=tmp_path / 'robot')


# --------------------------------------------------------------------------
# Consistency between the data files
# --------------------------------------------------------------------------

def test_every_joint_has_acceleration_limit_and_servo_entry(robot, cr5_data, paths):
    sim = yaml.safe_load((paths.mujoco_src / 'config/cr5_sim.yaml').read_text())
    for name in robot.joint_names:
        assert name in cr5_data['acceleration_limits']['values']
        assert name in sim['joints']


def test_pair_links_exist_in_urdf(robot, cr5_data):
    assert gc.arm_link_names(cr5_data) <= set(robot.links)


def test_ready_pose_matches_joint_count(robot):
    assert len(gc.READY) == len(robot.joints)


# --------------------------------------------------------------------------
# Command line used by CMake
# --------------------------------------------------------------------------

@pytest.mark.parametrize('target', gc.TARGETS)
def test_cli_target_succeeds_and_touches_the_stamp(tmp_path, paths, target):
    stamp = tmp_path / 'stamp'
    proc = subprocess.run(
        [sys.executable, str(paths.script), target,
         '--cell-share', str(paths.cell_share),
         '--robot-share', str(paths.robot_share),
         '--src', str(paths.mujoco_src),
         '--robot-urdf', str(paths.urdf),
         '--out', str(tmp_path / 'out'), '--stamp', str(stamp)],
        capture_output=True, text=True)
    assert proc.returncode == 0, proc.stderr
    assert stamp.exists()


def test_cli_missing_robot_share_is_a_usage_error(tmp_path, paths):
    proc = subprocess.run(
        [sys.executable, str(paths.script), 'moveit',
         '--cell-share', str(paths.cell_share), '--out', str(tmp_path)],
        capture_output=True, text=True)
    assert proc.returncode == 2
    assert '--robot-share' in proc.stderr


# --------------------------------------------------------------------------
# Fixture drift: compare with the real xacro when ROS is available
# --------------------------------------------------------------------------

@pytest.mark.skipif(shutil.which('xacro') is None, reason='xacro not available')
def test_real_urdf_matches_fixture(paths, robot):
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory('dobot_cr5_description')
    except Exception:
        pytest.skip('dobot_cr5_description is not installed')

    real = gc.load_robot(paths.robot_share, None)
    assert real.joint_names == robot.joint_names
    assert real.tool_parent == robot.tool_parent

    def numbers(text):
        return [float(v) for v in text.split()]

    for a, b in zip(real.joints, robot.joints):
        for field in ('xyz', 'rpy', 'axis', 'lower', 'upper', 'effort',
                      'velocity', 'damping', 'friction'):
            va, vb = numbers(getattr(a, field)), numbers(getattr(b, field))
            assert len(va) == len(vb), f'{a.name}.{field}'
            for x, y in zip(va, vb):
                assert math.isclose(x, y, rel_tol=1e-12, abs_tol=1e-15), (
                    f'{a.name}.{field} differs from the fixture')
    for name, link in real.links.items():
        other = robot.links[name]
        assert numbers(link.inertia or '0') == numbers(other.inertia or '0'), name
        assert numbers(link.com) == numbers(other.com), name
        assert link.visual == other.visual, name
