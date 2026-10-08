import copy
import itertools
import os
from xml.etree.ElementTree import (
    Element,
    ElementTree,
    SubElement,
    indent,
    register_namespace,
)

import yaml
from ament_index_python.packages import get_package_share_directory

PKG_ROBOT = 'dobot_description'
PKG_EOAT = 'eoat_description'
PKG_MECHANISM = 'mechanism_description'
PKG_MUJOCO = 'mujoco_system'
PKG_MOVEIT = 'cell_moveit_config'
PKG_CELL = 'cell_bringup'

ROBOT_ROOT = 'base_link'
FLANGE_LINK = 'flange'

# Must match the prefix used in MuJoCo: <attach ... prefix="grip_"/> inside
# the robot, which itself is attached with prefix="<robot>_".
GRIPPER_INFIX = 'grip_'

# Single source of truth for named poses (used by per-arm and all_arms states).
ROBOT_POSE: dict[str, list[float]] = {
    'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'ready': [0.0, 0.0, 1.5707, 0.0, -1.5707, 0.0]
}

HUMBLE_ADAPTERS = (
    'default_planner_request_adapters/AddTimeOptimalParameterization '
    'default_planner_request_adapters/ResolveConstraintFrames '
    'default_planner_request_adapters/FixWorkspaceBounds '
    'default_planner_request_adapters/FixStartStateBounds '
    'default_planner_request_adapters/FixStartStateCollision '
    'default_planner_request_adapters/FixStartStatePathConstraints'
)

ROBOT_CONFIG: list[dict[str, str | None]] = [
    {
        'robot': 'cr5',
        'prefix': 'r1',
        'robot_xyz': '-0.466 -0.652 0',
        'robot_rpy': '0 0 1.5708',
        'gripper': 'pgc_50_35',
        'gripper_xyz': '0 0 0',
        'gripper_rpy': '0 0 0'

    },
    {
        'robot': 'cr5',
        'prefix': 'r2',
        'robot_xyz': '0.152 0.466 0',
        'robot_rpy': '0 0 0',
        'gripper': None,
        'gripper_xyz': None,
        'gripper_rpy': None

    },
    {
        'robot': 'cr5',
        'prefix': 'r3',
        'robot_xyz': '1.148 0.466 0',
        'robot_rpy': '0 0 0',
        'gripper': None,
        'gripper_xyz': None,
        'gripper_rpy': None

    },
    {
        'robot': 'cr5',
        'prefix': 'r4',
        'robot_xyz': '2.148 0.466 0',
        'robot_rpy': '0 0 0',
        'gripper': None,
        'gripper_xyz': None,
        'gripper_rpy': None

    },
    {
        'robot': 'cr5',
        'prefix': 'r5',
        'robot_xyz': '3.148 0.466 0',
        'robot_rpy': '0 0 0',
        'gripper': None,
        'gripper_xyz': None,
        'gripper_rpy': None

    },
    {
        'robot': 'cr5',
        'prefix': 'r6',
        'robot_xyz': '3.148 0.466 3.795',
        'robot_rpy': '0 3.1416 0',
        'gripper': None,
        'gripper_xyz': None,
        'gripper_rpy': None

    },
    {
        'robot': 'cr5',
        'prefix': 'r7',
        'robot_xyz': '0.770 -1.152 3.795',
        'robot_rpy': '0 3.1416 -1.5708',
        'gripper': None,
        'gripper_xyz': None,
        'gripper_rpy': None

    }
]


def _prefixes() -> list[str]:
    """Robot prefixes in a deterministic order (sets are random per run)."""
    return sorted({item['prefix'] for item in ROBOT_CONFIG})  # type: ignore


def _gripper_prefixes() -> list[str]:
    return sorted(
        item['prefix'] for item in ROBOT_CONFIG  # type: ignore
        if item['gripper'] is not None)


def _write_xml(tree: ElementTree, path: str) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    indent(tree, space='    ', level=0)
    tree.write(path, encoding='utf-8', xml_declaration=True)


def _write_yaml(data: dict, path: str) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', encoding='utf-8') as f:
        yaml.dump(data, f, sort_keys=False, indent=2)


def _load_disabled_pairs() -> list[dict[str, str]]:
    """Single source for collision pairs: dobot_description/config/cr5.yaml."""
    path = os.path.join(
        get_package_share_directory(PKG_ROBOT), 'config', 'cr5.yaml')
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)['collision']['disabled_pairs']


class GenerateSRDF:
    def __init__(self) -> None:
        pkg: str = get_package_share_directory(PKG_MOVEIT)
        self._save_path: str = os.path.join(
            pkg, 'config', 'cell.srdf'
        )
        self._disabled_pairs = _load_disabled_pairs()

    def _generate_gripper_srdf(self, parent: Element, prefix: str) -> None:
        grip = f'{prefix}_{GRIPPER_INFIX}'

        SubElement(
            parent, 'disable_collisions', link1=f'{prefix}_link_5',
            link2=f'{grip}gripper_base', reason='Adjacent')

        SubElement(
            parent, 'disable_collisions', link1=f'{prefix}_link_6',
            link2=f'{grip}gripper_base', reason='Adjacent')

        SubElement(
            parent, 'disable_collisions', link1=f'{grip}gripper_base',
            link2=f'{grip}finger_1', reason='Adjacent')

        SubElement(
            parent, 'disable_collisions', link1=f'{grip}gripper_base',
            link2=f'{grip}finger_2', reason='Adjacent')

        SubElement(
            parent, 'disable_collisions', link1=f'{grip}finger_1',
            link2=f'{grip}finger_2', reason='Adjacent')

    def _generate_robot_srdf(self, parent: Element, prefix: str) -> None:
        group: Element = SubElement(parent, 'group', name=f'{prefix}_arm')

        SubElement(
            group, 'chain', base_link=f'{prefix}_{ROBOT_ROOT}',
            tip_link=f'{prefix}_{FLANGE_LINK}')

        # Per-arm states now come from ROBOT_POSE (previously 'ready' was
        # named 'home' and had opposite signs to ROBOT_POSE['ready']).
        for state_name, values in ROBOT_POSE.items():
            group_state: Element = SubElement(
                parent, 'group_state', name=state_name,
                group=f'{prefix}_arm')

            for i, value in enumerate(values, start=1):
                SubElement(
                    group_state, 'joint',
                    name=f'{prefix}_joint_{i}', value=str(value))

        for pair in self._disabled_pairs:
            SubElement(
                parent, 'disable_collisions',
                link1=f'{prefix}_{pair["link1"]}',
                link2=f'{prefix}_{pair["link2"]}',
                reason=pair.get('reason', 'Unverified'))

    def _generate_robot_group(self, parent: Element, prefix: list[str]) -> None:
        group: Element = SubElement(parent, 'group', name='all_arms')

        for item in prefix:
            SubElement(group, 'group', name=f'{item}_arm')

        for key, value in ROBOT_POSE.items():
            group_state: Element = SubElement(
                parent, 'group_state', name=key, group='all_arms')

            for item in prefix:
                for i in range(len(value)):
                    SubElement(
                        group_state, 'joint', name=f'{item}_joint_{i+1}',
                        value=str(value[i]))

    def generate(self) -> None:
        root: Element = Element('robot', name='cell')

        robot_prefix: list[str] = _prefixes()

        # Arm groups first, then the group that references them.
        for config in ROBOT_CONFIG:
            self._generate_robot_srdf(
                root, prefix=config['prefix'])  # type: ignore
            if config['gripper'] is not None:
                self._generate_gripper_srdf(
                    root, prefix=config['prefix'])  # type: ignore

        self._generate_robot_group(root, robot_prefix)

        _write_xml(ElementTree(root), self._save_path)


class GenerateXacro:
    def __init__(self) -> None:
        pkg_main: str = get_package_share_directory(PKG_MUJOCO)
        self._save_path_main: str = os.path.join(
            pkg_main, 'urdf', 'cell_mujoco.urdf.xacro'
        )

        pkg_cell: str = get_package_share_directory(PKG_CELL)
        self._save_path_cell: str = os.path.join(
            pkg_cell, 'urdf', 'cell.urdf.xacro'
        )

    def __generate_argument(self, parent: Element, name: str, default: str) -> None:
        SubElement(parent, 'xacro:arg', name=name, default=default)

    def __attach_world(self, parent: Element, name: str, link: str, prefix: str, xyz: str, rpy: str) -> None:
        joint: Element = SubElement(parent, 'joint', name=name, type='fixed')
        SubElement(joint, 'parent', link=link)
        SubElement(joint, 'child', link=f'{prefix}_{ROBOT_ROOT}')
        SubElement(joint, 'origin', xyz=xyz, rpy=rpy)

    def __insert_gripper(self, parent: Element, prefix: str, xyz: str, rpy: str) -> None:
        # Prefix must produce r1_grip_gripper_base / r1_grip_slider_1 / ...
        gripper: Element = SubElement(
            parent, 'xacro:pgc_50_35', {
                'prefix': f'{prefix}_{GRIPPER_INFIX}',
                'parent': f'{prefix}_{FLANGE_LINK}'
            })
        SubElement(gripper, 'origin', xyz=xyz, rpy=rpy)

    def __add_include(self, parent: Element, filename: str) -> None:
        SubElement(parent, 'xacro:include', filename=filename)

    def __generate_ros2_control(self, parent: Element) -> Element:
        control: Element = SubElement(
            parent, 'ros2_control', name='cell_system', type='system')

        hardware: Element = SubElement(control, 'hardware')
        plugin: Element = SubElement(hardware, 'plugin')
        plugin.text = 'mujoco_ros2_control/MujocoSystemInterface'
        param_1: Element = SubElement(hardware, 'param', name='mujoco_model')
        param_1.text = '$(arg mujoco_model)'
        param_2: Element = SubElement(
            hardware, 'param', name='auto_register_cameras')
        param_2.text = 'false'

        return control

    def __generate_cr5_interface_robot(self, parent: Element, prefix: list[str]) -> None:
        for item, i in itertools.product(prefix, range(1, 7)):
            SubElement(
                parent, 'xacro:cr5_joint_interface',
                joint_name=f'{item}_joint_{i}')

    def __generate_interface_gripper(self, parent: Element, prefix: list[str]) -> None:
        for item in prefix:
            SubElement(
                parent, 'xacro:cr5_joint_interface',
                joint_name=f'{item}_{GRIPPER_INFIX}slider_1')

    def _generate_cell(self) -> None:
        register_namespace('xacro', 'http://www.ros.org/wiki/xacro')

        root: Element = Element('robot', {
            'xmlns:xacro': 'http://www.ros.org/wiki/xacro',
            'name': 'cell'
        })

        self.__add_include(
            root, f'$(find {PKG_ROBOT})/urdf/cr5/cr5_macro.xacro')

        self.__add_include(
            root, f'$(find {PKG_EOAT})/urdf/pgc/pgc_50_35/pgc_50_35_macro.xacro')

        SubElement(root, 'link', name='world')

        for robot in ROBOT_CONFIG:
            SubElement(root, 'xacro:cr5_robot', prefix=f'{robot["prefix"]}_')

            self.__attach_world(
                root, f'{robot["prefix"]}_base_joint', 'world',
                robot["prefix"], xyz=robot['robot_xyz'], rpy=robot['robot_rpy'])  # type: ignore

            if robot['gripper']:
                self.__insert_gripper(
                    root, robot['prefix'], xyz=robot['gripper_xyz'], rpy=robot['gripper_rpy'])  # type: ignore

        _write_xml(ElementTree(root), self._save_path_cell)

    def _generate_main(self) -> None:
        register_namespace('xacro', 'http://www.ros.org/wiki/xacro')

        root: Element = Element('robot', {
            'xmlns:xacro': 'http://www.ros.org/wiki/xacro',
            'name': 'cell'
        })

        robot_prefix: list[str] = _prefixes()
        gripper: list[str] = _gripper_prefixes()

        self.__generate_argument(
            root, 'mujoco_model',
            f'$(find {PKG_MUJOCO})/scenes/cell.xml')

        self.__add_include(
            root, f'$(find {PKG_CELL})/urdf/cell.urdf.xacro')

        self.__add_include(
            root, f'$(find {PKG_ROBOT})/urdf/cr5/cr5_ros2_control.xacro')

        # Joint interfaces must live INSIDE <ros2_control>, not under <robot>.
        control: Element = self.__generate_ros2_control(root)

        self.__generate_cr5_interface_robot(control, robot_prefix)

        if gripper:
            self.__generate_interface_gripper(control, gripper)

        _write_xml(ElementTree(root), self._save_path_main)

    def _generate_controllers(self) -> None:
        path = os.path.join(
            get_package_share_directory(PKG_MUJOCO),
            'config', 'cell_controllers.yaml')

        manager: dict = {
            'update_rate': 500,
            'joint_state_broadcaster': {
                'type': 'joint_state_broadcaster/JointStateBroadcaster'},
        }
        params: dict = {}

        for item in ROBOT_CONFIG:
            p = item['prefix']

            manager[f'{p}_arm_controller'] = {
                'type': 'joint_trajectory_controller/JointTrajectoryController'}
            params[f'{p}_arm_controller'] = {'ros__parameters': {
                'joints': [f'{p}_joint_{i}' for i in range(1, 7)],
                'command_interfaces': ['position'],
                'state_interfaces': ['position', 'velocity'],
                'state_publish_rate': 50.0,
                'action_monitor_rate': 20.0,
                'allow_partial_joints_goal': False,
                'allow_nonzero_velocity_at_trajectory_end': False,
                'constraints': {
                    'stopped_velocity_tolerance': 0.01,
                    'goal_time': 0.0,
                },
            }}

            if item['gripper']:
                manager[f'{p}_gripper_controller'] = {
                    'type': 'forward_command_controller/ForwardCommandController'}
                params[f'{p}_gripper_controller'] = {'ros__parameters': {
                    'joints': [f'{p}_{GRIPPER_INFIX}slider_1'],
                    'interface_name': 'position',
                }}

        data = {'controller_manager': {'ros__parameters': manager}, **params}
        _write_yaml(data, path)

    def generate(self) -> None:
        self._generate_cell()
        self._generate_main()
        self._generate_controllers()


class GenerateConfig:
    def __init__(self) -> None:
        pkg: str = get_package_share_directory(PKG_MOVEIT)
        self._save_path: str = os.path.join(
            pkg, 'config'
        )
        template_path: str = os.path.join(
            pkg, 'templates'
        )
        with open(f'{template_path}/cr5_joint_limits.yaml', 'r') as data:
            self.__cr5_joint_limit: dict = yaml.safe_load(data)

        with open(f'{template_path}/kinematics.yaml', 'r') as data:
            self.__cr5_kinematics: dict = yaml.safe_load(data)

        with open(f'{template_path}/moveit_controllers.yaml', 'r') as data:
            self.__cr5_moveit_controls: dict = yaml.safe_load(data)

        with open(f'{template_path}/ompl_planning.yaml', 'r') as data:
            self.__cr5_ompl: dict = yaml.safe_load(data)

        with open(f'{template_path}/pilz_cartesian_limits.yaml', 'r') as data:
            self.__cr5_pilz: dict = yaml.safe_load(data)

        self._prefix: list[str] = _prefixes()

    def _generate_joint_limits(self) -> None:
        joint_limit: dict = {}

        template: dict = self.__cr5_joint_limit['joint_limits']

        for _prefix in self._prefix:
            for joint_name, limits in template.items():
                joint_limit[f'{_prefix}_{joint_name}'] = copy.deepcopy(limits)

        self.__cr5_joint_limit['joint_limits'] = joint_limit

        _write_yaml(
            self.__cr5_joint_limit,
            f'{self._save_path}/cell_joint_limits.yaml')

    def _generate_kinematics(self) -> None:
        kinematics = {
            f'{_prefix}_arm': copy.deepcopy(self.__cr5_kinematics['arm'])
            for _prefix in self._prefix
        }

        _write_yaml(kinematics, f'{self._save_path}/cell_kinematics.yaml')

    def _generate_moveit_controls(self) -> None:
        manager = self.__cr5_moveit_controls['moveit_simple_controller_manager']
        ctrl = manager['arm_controller']
        ctrl_name = [f'{_prefix}_arm_controller' for _prefix in self._prefix]
        updated = {'controller_names': ctrl_name}
        _joints = ctrl['joints']

        for _prefix in self._prefix:
            joints = [f'{_prefix}_{joint}' for joint in _joints]

            controller_config = copy.deepcopy(ctrl)
            controller_config['joints'] = joints

            updated[f'{_prefix}_arm_controller'] = controller_config

        self.__cr5_moveit_controls['moveit_simple_controller_manager'] = updated

        _write_yaml(
            self.__cr5_moveit_controls,
            f'{self._save_path}/cell_moveit_controllers.yaml')

    def _generate_ompl(self) -> None:
        arm = self.__cr5_ompl.pop('arm')

        # deepcopy avoids YAML anchors/aliases (&id001 / *id001) in output.
        for _prefix in self._prefix:
            self.__cr5_ompl[f'{_prefix}_arm'] = copy.deepcopy(arm)

        if len(self._prefix) > 1:
            self.__cr5_ompl['all_arms'] = copy.deepcopy(arm)

        _write_yaml(
            self.__cr5_ompl, f'{self._save_path}/cell_ompl_planning.yaml')

    def _generate_pilz(self) -> None:
        _write_yaml(
            self.__cr5_pilz, f'{self._save_path}/pilz_cartesian_limits.yaml')

    def _generate_cell(self) -> None:
        config: list[dict[str, object]] = []

        config.extend(
            {
                'name': robot['prefix'],  # type: ignore
                'gripper': robot.get('gripper'),
            }
            for robot in ROBOT_CONFIG  # type: ignore
        )

        _write_yaml({'robots': config}, f'{self._save_path}/cell.yaml')

    def generate(self) -> None:
        self._generate_joint_limits()
        self._generate_kinematics()
        self._generate_moveit_controls()
        self._generate_ompl()
        self._generate_pilz()
        self._generate_cell()


def main() -> None:
    srdf = GenerateSRDF()
    srdf.generate()

    config = GenerateConfig()
    config.generate()

    xacro = GenerateXacro()
    xacro.generate()


if __name__ == '__main__':
    main()
