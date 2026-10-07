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
        'robot_xyz': '0 -0.5 0',
        'robot_rpy': '0 0 0',
        'gripper': 'pgc_50_35',
        'gripper_xyz': '0 0 0',
        'gripper_rpy': '0 0 0'

    },
    {
        'robot': 'cr5',
        'prefix': 'r2',
        'robot_xyz': '0 0.5 0',
        'robot_rpy': '0 0 0',
        'gripper': None,
        'gripper_xyz': None,
        'gripper_rpy': None

    }
]


class GenerateSRDF:
    def __init__(self) -> None:
        pkg: str = get_package_share_directory(PKG_MOVEIT)
        self._save_path: str = os.path.join(
            pkg, 'config', 'cell.srdf'
        )

    def _generate_gripper_srdf(self, parent: Element, prefix: str) -> None:
        SubElement(
            parent, 'disable_collisions', link1=f'{prefix}_link_5',
            link2=f'{prefix}_grip_gripper_base', reason='Adjacent')

        SubElement(
            parent, 'disable_collisions', link1=f'{prefix}_link_6',
            link2=f'{prefix}_grip_gripper_base', reason='Adjacent')

        SubElement(
            parent, 'disable_collisions', link1=f'{prefix}_grip_gripper_base',
            link2=f'{prefix}_grip_finger_1', reason='Adjacent')

        SubElement(
            parent, 'disable_collisions', link1=f'{prefix}_grip_gripper_base',
            link2=f'{prefix}_grip_finger_2', reason='Adjacent')

        SubElement(
            parent, 'disable_collisions', link1=f'{prefix}_grip_finger_1',
            link2=f'{prefix}_grip_finger_2', reason='Adjacent')

    def _generate_robot_srdf(self, parent: Element, prefix: str) -> None:
        group: Element = SubElement(parent, 'group', name=f'{prefix}_arm')

        SubElement(
            group, 'chain', base_link=f'{prefix}_{ROBOT_ROOT}',
            tip_link=f'{prefix}_{FLANGE_LINK}')

        group_state_home: Element = SubElement(
            parent, 'group_state', name='home',
            group=f'{prefix}_arm')

        for i in range(1, 7):
            SubElement(
                group_state_home, 'joint',
                name=f'{prefix}_joint_{i}', value='0')

        group_state_ready: Element = SubElement(
            parent, 'group_state', name='home',
            group=f'{prefix}_arm')

        for i in range(1, 7):
            value = '0'

            if i == 3:
                value = '-1.5707'
            elif i == 5:
                value = '1.5707'

            SubElement(
                group_state_ready, 'joint', name=f'{prefix}_joint_{i}',
                value=value)

        SubElement(
            parent, 'disable_collisions', link1=f'{prefix}_{ROBOT_ROOT}',
            link2=f'{prefix}_link_1', reason='Adjacent')

        SubElement(
            parent, 'disable_collisions', link1=f'{prefix}_link_1',
            link2=f'{prefix}_link_2', reason='Adjacent')

        SubElement(
            parent, 'disable_collisions', link1=f'{prefix}_link_1',
            link2=f'{prefix}_link_3', reason='Unverified')

        SubElement(
            parent, 'disable_collisions', link1=f'{prefix}_link_1',
            link2=f'{prefix}_link_4', reason='Unverified')

        SubElement(
            parent, 'disable_collisions', link1=f'{prefix}_link_2',
            link2=f'{prefix}_link_3', reason='Adjacent')

        SubElement(
            parent, 'disable_collisions', link1=f'{prefix}_link_3',
            link2=f'{prefix}_link_4', reason='Adjacent')

        SubElement(
            parent, 'disable_collisions', link1=f'{prefix}_link_4',
            link2=f'{prefix}_link_5', reason='Adjacent')

        SubElement(
            parent, 'disable_collisions', link1=f'{prefix}_link_4',
            link2=f'{prefix}_link_6', reason='Unverified')

        SubElement(
            parent, 'disable_collisions', link1=f'{prefix}_link_5',
            link2=f'{prefix}_link_6', reason='Adjacent')

    def _generate_robot_group(self, parent: Element, prefix: set[str]) -> None:
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

        robot_prefix: set[str] = {
            item['prefix'] for item in ROBOT_CONFIG}  # type: ignore
        self._generate_robot_group(root, robot_prefix)

        for config in ROBOT_CONFIG:
            self._generate_robot_srdf(
                root, prefix=config['prefix'])  # type: ignore
            if config['gripper'] is not None:
                self._generate_gripper_srdf(
                    root, prefix=config['prefix'])  # type: ignore

        tree: ElementTree = ElementTree(root)
        indent(tree, space='    ', level=0)

        tree.write(self._save_path, encoding='utf-8', xml_declaration=True)


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
        gripper: Element = SubElement(
            parent, 'xacro:pgc_50_35', {
                'prefix': prefix,
                'parent': f'{prefix}_{FLANGE_LINK}'
            })
        SubElement(gripper, 'origin', xyz=xyz, rpy=rpy)

    def __add_include(self, parent: Element, filename: str) -> None:
        SubElement(parent, 'xacro:include', filename=filename)

    def __generate_ros2_control(self, parent: Element) -> None:
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

    def __generate_cr5_interface_robot(self, parent: Element, prefix: set[str]) -> None:
        for item, i in itertools.product(prefix, range(1, 7)):
            SubElement(
                parent, 'xacro:cr5_joint_interface',
                joint_name=f'{item}_joint_{i}')

    def __generate_interface_gripper(self, parent: Element, prefix: set[str]) -> None:
        for item in prefix:
            SubElement(
                parent, 'xacro:cr5_joint_interface',
                joint_name=f'{item}_grip_slider_1')

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

        tree: ElementTree = ElementTree(root)
        indent(tree, space='    ', level=0)
        tree.write(
            self._save_path_cell,
            encoding='utf-8', xml_declaration=True)

    def _generate_main(self) -> None:
        register_namespace('xacro', 'http://www.ros.org/wiki/xacro')

        root: Element = Element('robot', {
            'xmlns:xacro': 'http://www.ros.org/wiki/xacro',
            'name': 'cell'
        })

        robot_prefix: set[str] = {
            item['prefix'] for item in ROBOT_CONFIG}  # type: ignore

        gripper: set[str | None] | None = {
            item['prefix'] for item in ROBOT_CONFIG if item['gripper'] is not None} or None

        self.__generate_argument(
            root, 'mujoco_model',
            f'$(find {PKG_MUJOCO})/scenes/cell.xml')

        self.__add_include(
            root, f'$(find {PKG_CELL})/urdf/cell.urdf.xacro')

        self.__add_include(
            root, f'$(find {PKG_ROBOT})/urdf/cr5/cr5_ros2_control.xacro')

        self.__generate_ros2_control(root)

        self.__generate_cr5_interface_robot(root, robot_prefix)

        if gripper is not None:
            self.__generate_interface_gripper(root, gripper)  # type: ignore

        tree: ElementTree = ElementTree(root)
        indent(tree, space='    ', level=0)
        tree.write(
            self._save_path_main,
            encoding='utf-8', xml_declaration=True)

    def generate(self) -> None:
        self._generate_cell()
        self._generate_main()


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

        self._prefix: set[str] = {  # type: ignore
            item['prefix'] for item in ROBOT_CONFIG}  # type: ignore

    def _generate_joint_limits(self) -> None:
        joint_limit: dict = {}

        template: dict = self.__cr5_joint_limit['joint_limits']

        for joint_name, limits in template.items():
            for _prefix in self._prefix:
                joint_limit[f'{_prefix}_{joint_name}'] = copy.deepcopy(limits)

        self.__cr5_joint_limit['joint_limits'] = joint_limit

        with open(f'{self._save_path}/cell_joint_limits.yaml', 'w', encoding='utf-8') as f:
            yaml.dump(self.__cr5_joint_limit, f, sort_keys=False, indent=2)

    def _generate_kinematics(self) -> None:
        kinematics = {
            f'{_prefix}_arm': self.__cr5_kinematics['arm']
            for _prefix in self._prefix
        }

        with open(f'{self._save_path}/cell_kinematics.yaml', 'w', encoding='utf-8') as f:
            yaml.dump(kinematics, f, sort_keys=False, indent=2)

    def _generate_moveit_controls(self) -> None:
        manager = self.__cr5_moveit_controls['moveit_simple_controller_manager']
        ctrl = manager['arm_controller']
        ctrl_name = [f'{_prefix}_arm_controller' for _prefix in self._prefix]
        updated = {'controller_names': ctrl_name}
        _joints = ctrl.pop('joints')

        for _prefix in self._prefix:
            joints = [f'{_prefix}_{joint}' for joint in _joints]

            controller_config = copy.deepcopy(ctrl)
            controller_config['joints'] = joints

            updated[f'{_prefix}_arm_controller'] = controller_config

        self.__cr5_moveit_controls['moveit_simple_controller_manager'] = updated

        with open(f'{self._save_path}/cell_moveit_controllers.yaml', 'w', encoding='utf-8') as f:
            yaml.dump(self.__cr5_moveit_controls, f, sort_keys=False, indent=2)

    def _generate_ompl(self) -> None:
        arm = self.__cr5_ompl.pop('arm')

        for _prefix in self._prefix:
            self.__cr5_ompl[f'{_prefix}_arm'] = arm

        if len(self._prefix) > 1:
            self.__cr5_ompl['all_arms'] = arm

        with open(f'{self._save_path}/cell_ompl_planning.yaml', 'w', encoding='utf-8') as f:
            yaml.dump(self.__cr5_ompl, f, sort_keys=False, indent=2)

    def _generate_pilz(self) -> None:
        with open(f'{self._save_path}/pilz_cartesian_limits.yaml', 'w', encoding='utf-8') as f:
            yaml.dump(self.__cr5_pilz, f, sort_keys=False, indent=2)

    def _generate_cell(self) -> None:
        config: list[dict[str, object]] = []

        config.extend(
            {
                'name': robot['prefix'],  # type: ignore
                'gripper': robot.get('gripper'),
            }
            for robot in ROBOT_CONFIG  # type: ignore
        )

        data = {
            'robots': config
        }

        with open(f'{self._save_path}/cell.yaml', 'w', encoding='utf-8') as f:
            yaml.dump(data, f, sort_keys=False, indent=2)

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


main()
