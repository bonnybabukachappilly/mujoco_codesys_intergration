"""Generate the cell's MoveIt / ros2_control / xacro configuration.

Run by CMake at build time (see CMakeLists.txt), or by hand:

    python3 generate_config.py --templates ../templates \
        --cr5-config <path>/dobot_description/config/cr5.yaml --out /tmp/gen

Output layout, relative to --out:
    config/  cell.srdf cell.yaml cell_controllers.yaml cell_joint_limits.yaml
             cell_kinematics.yaml cell_moveit_controllers.yaml
             cell_ompl_planning.yaml pilz_cartesian_limits.yaml
    urdf/    cell.urdf.xacro cell_mujoco.urdf.xacro
"""
import argparse
import copy
import itertools
from pathlib import Path
from xml.etree.ElementTree import Element, ElementTree, SubElement, indent

import yaml

PKG_ROBOT = 'dobot_description'
PKG_EOAT = 'eoat_description'
PKG_MUJOCO = 'mujoco_system'
PKG_MOVEIT = 'cell_moveit_config'  # also owns the generated xacro now

ROBOT_ROOT = 'base_link'
FLANGE_LINK = 'flange'

# Must match the prefix used in MuJoCo: <attach ... prefix="grip_"/> inside
# the robot, which itself is attached with prefix="<robot>_".
GRIPPER_INFIX = 'grip_'

# Keep in sync with GENERATED_FILES in CMakeLists.txt.
EXPECTED_OUTPUTS = (
    'config/cell.srdf',
    'config/cell.yaml',
    'config/cell_controllers.yaml',
    'config/cell_joint_limits.yaml',
    'config/cell_kinematics.yaml',
    'config/cell_moveit_controllers.yaml',
    'config/cell_ompl_planning.yaml',
    'config/pilz_cartesian_limits.yaml',
    'urdf/cell.urdf.xacro',
    'urdf/cell_mujoco.urdf.xacro',
)

# Single source of truth for named poses (used by per-arm and all_arms states).
ROBOT_POSE: dict[str, list[float]] = {
    'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'ready': [0.0, 0.0, 1.5707, 0.0, -1.5707, 0.0]
}

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
    """Robot prefixes in ROBOT_CONFIG order (deterministic, r10 after r9)."""
    return list(dict.fromkeys(
        item['prefix'] for item in ROBOT_CONFIG))  # type: ignore


def _gripper_prefixes() -> list[str]:
    return [
        item['prefix'] for item in ROBOT_CONFIG  # type: ignore
        if item['gripper'] is not None]


def _write_xml(tree: ElementTree, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    indent(tree, space='    ', level=0)
    tree.write(path, encoding='utf-8', xml_declaration=True)


def _write_yaml(data: dict, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, 'w', encoding='utf-8') as f:
        yaml.dump(data, f, sort_keys=False, indent=2)


def _load_yaml(path: Path) -> dict:
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


class GenerateSRDF:
    def __init__(self, out: Path, cr5_config: Path) -> None:
        self._save_path: Path = out / 'config' / 'cell.srdf'
        # Single source for collision pairs: dobot_description/config/cr5.yaml
        self._disabled_pairs: list[dict[str, str]] = (
            _load_yaml(cr5_config)['collision']['disabled_pairs'])

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
    def __init__(self, out: Path) -> None:
        self._out: Path = out
        self._save_path_main: Path = out / 'urdf' / 'cell_mujoco.urdf.xacro'
        self._save_path_cell: Path = out / 'urdf' / 'cell.urdf.xacro'

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

    @staticmethod
    def __new_root() -> Element:
        return Element('robot', {
            'xmlns:xacro': 'http://www.ros.org/wiki/xacro',
            'name': 'cell'
        })

    def _generate_cell(self) -> None:
        root: Element = self.__new_root()

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
        root: Element = self.__new_root()

        robot_prefix: list[str] = _prefixes()
        gripper: list[str] = _gripper_prefixes()

        self.__generate_argument(
            root, 'mujoco_model',
            f'$(find {PKG_MUJOCO})/scenes/cell.xml')

        # cell.urdf.xacro is generated into this same package now
        # (it used to live in cell_bringup, which created a package cycle).
        self.__add_include(
            root, f'$(find {PKG_MOVEIT})/urdf/cell.urdf.xacro')

        self.__add_include(
            root, f'$(find {PKG_ROBOT})/urdf/cr5/cr5_ros2_control.xacro')

        # Joint interfaces must live INSIDE <ros2_control>, not under <robot>.
        control: Element = self.__generate_ros2_control(root)

        self.__generate_cr5_interface_robot(control, robot_prefix)

        if gripper:
            self.__generate_interface_gripper(control, gripper)

        _write_xml(ElementTree(root), self._save_path_main)

    def _generate_controllers(self) -> None:
        path = self._out / 'config' / 'cell_controllers.yaml'

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
    def __init__(self, out: Path, templates: Path) -> None:
        self._save_path: Path = out / 'config'

        self.__cr5_joint_limit: dict = _load_yaml(
            templates / 'cr5_joint_limits.yaml')
        self.__cr5_kinematics: dict = _load_yaml(
            templates / 'kinematics.yaml')
        self.__cr5_moveit_controls: dict = _load_yaml(
            templates / 'moveit_controllers.yaml')
        self.__cr5_ompl: dict = _load_yaml(
            templates / 'ompl_planning.yaml')
        self.__cr5_pilz: dict = _load_yaml(
            templates / 'pilz_cartesian_limits.yaml')

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
            self._save_path / 'cell_joint_limits.yaml')

    def _generate_kinematics(self) -> None:
        kinematics = {
            f'{_prefix}_arm': copy.deepcopy(self.__cr5_kinematics['arm'])
            for _prefix in self._prefix
        }

        _write_yaml(kinematics, self._save_path / 'cell_kinematics.yaml')

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
            self._save_path / 'cell_moveit_controllers.yaml')

    def _generate_ompl(self) -> None:
        arm = self.__cr5_ompl.pop('arm')

        # deepcopy avoids YAML anchors/aliases (&id001 / *id001) in output.
        for _prefix in self._prefix:
            self.__cr5_ompl[f'{_prefix}_arm'] = copy.deepcopy(arm)

        if len(self._prefix) > 1:
            self.__cr5_ompl['all_arms'] = copy.deepcopy(arm)

        _write_yaml(
            self.__cr5_ompl, self._save_path / 'cell_ompl_planning.yaml')

    def _generate_pilz(self) -> None:
        _write_yaml(
            self.__cr5_pilz, self._save_path / 'pilz_cartesian_limits.yaml')

    def _generate_cell(self) -> None:
        config: list[dict[str, object]] = []

        config.extend(
            {
                'name': robot['prefix'],  # type: ignore
                'gripper': robot.get('gripper'),
            }
            for robot in ROBOT_CONFIG  # type: ignore
        )

        _write_yaml({'robots': config}, self._save_path / 'cell.yaml')

    def generate(self) -> None:
        self._generate_joint_limits()
        self._generate_kinematics()
        self._generate_moveit_controls()
        self._generate_ompl()
        self._generate_pilz()
        self._generate_cell()


def main() -> None:
    parser = argparse.ArgumentParser(description='Generate cell config.')
    parser.add_argument('--templates', type=Path, required=True,
                        help='directory with the *.yaml templates')
    parser.add_argument('--cr5-config', type=Path, required=True,
                        help='dobot_description/config/cr5.yaml')
    parser.add_argument('--out', type=Path, required=True,
                        help='output directory (config/ and urdf/ created)')
    args = parser.parse_args()

    GenerateSRDF(args.out, args.cr5_config).generate()
    GenerateConfig(args.out, args.templates).generate()
    GenerateXacro(args.out).generate()

    if missing := [
        f for f in EXPECTED_OUTPUTS if not (args.out / f).is_file()
    ]:
        raise SystemExit(f'generator did not produce: {missing}')


if __name__ == '__main__':
    main()
