import argparse
from pathlib import Path, PosixPath
from typing import Any
from xml.etree.ElementTree import ElementTree, parse

import yaml
from generator.controller_generator import MoveitControllerGenerator
from generator.joint_limit_generator import JointLimitGenerator
from generator.kinematics_generator import KinematicsGenerator
from generator.models import (
    EOAT,
    CollisionDisabled,
    EOATJoint,
    EOATPose,
    PackageShare,
    Robot,
    RobotJoint,
    RobotPose,
    StationConfigurations,
    StationModel,
)
from generator.srdf_generator import SRDFGenerator


class GenerateStation:
    __slots__: tuple[str, ...] = (
        '_config',
        '_configurations',
        '_eoat',
        '_models',
        '_output_path',
        '_packages',
        '_robots'
    )

    def __init__(self, config_file: PosixPath, packages: PackageShare, output_path: PosixPath) -> None:
        self._models: list[StationModel] = []
        self._configurations: list[StationConfigurations] = []
        self._robots: list[Robot] = []
        self._eoat: list[EOAT] = []

        self._packages: PackageShare = packages
        self._output_path: PosixPath = output_path

        self._config: dict[str, Any] = self._parse_config(config_file)
        self._generate_elements()

        SRDFGenerator(
            config=self._configurations,
            get_robot=self._get_robot,
            get_eoat=self._get_eoat,
            output_path=self._output_path
        ).generate()

        MoveitControllerGenerator(
            config=self._configurations,
            get_robot=self._get_robot,
            output_path=self._output_path
        ).generate()

        JointLimitGenerator(
            config=self._configurations,
            get_robot=self._get_robot,
            output_path=self._output_path
        ).generate()

        KinematicsGenerator(
            config=self._configurations,
            output_path=self._output_path
        ).generate()

    def _get_robot(self, model: str) -> Robot:
        for robot in self._robots:
            if robot.model == model:
                return robot

        raise RuntimeError(
            'Unable to find the robot model. Please register in config.')

    def _get_eoat(self, model: str) -> EOAT:
        for eoat in self._eoat:
            if eoat.model == model:
                return eoat

        raise RuntimeError(
            'Unable to find the eoat model. Please register in config.')

    def _parse_config(self, config_file: PosixPath) -> dict[str, Any]:
        with open(config_file, 'r') as data:
            config_data: dict[str, Any] = yaml.safe_load(data)

        for model in config_data['models']:
            _mod: dict[str, Any] = config_data['models'][model]
            _ros: dict[str, str] = _mod['ros']
            _mujoco: dict[str, str] = _mod['mujoco']

            self._models.append(
                StationModel(
                    name=model,
                    make=_mod['make'],
                    type=_mod['type'],
                    ros_pkg=_ros['package'],
                    ros_descriptor=_ros['descriptor'],
                    mujoco_pkg=_mujoco['package'],
                    mujoco_descriptor=_mujoco['descriptor']
                )
            )

        for robot in config_data['robots']:
            _support: dict[str, str] = robot['support']
            _eoat: str | None = robot.get('eoat')
            _eoat_xyz: str | None = None
            _eoat_rpy: str | None = None

            if _eoat:
                _mount: dict[str, str] = robot.get('mount') or {}
                _eoat_xyz = _mount.get('xyz') or '0, 0, 0'
                _eoat_rpy = _mount.get('rpy') or '0, 0, 0'

            self._configurations.append(
                StationConfigurations(
                    model=robot['model'],
                    prefix=robot['id'],
                    robot_xyz=_support['xyz'],
                    robot_rpy=_support['rpy'],
                    eoat=_eoat,
                    eoat_xyz=_eoat_xyz,
                    eoat_rpy=_eoat_rpy,
                )
            )

        return config_data

    def _generate_elements(self) -> None:
        for model in self._models:
            if model.type == 'robot':
                self._generate_robot(model)

            elif model.type == 'eoat':
                self._generate_eoat(model)

    def _generate_eoat(self, model: StationModel) -> None:
        root: PosixPath = self._packages.eoat
        file: PosixPath = root / model.ros_descriptor

        with open(file, 'r') as data:
            eoat = yaml.safe_load(data)

        xacro_file: PosixPath = root / eoat['xacro']['file']
        xacro_tree: ElementTree = parse(xacro_file)  # type: ignore

        mujoco_root: PosixPath = self._packages.mujoco
        mujoco_file: PosixPath = mujoco_root / eoat['mujoco']['file']
        mujoco_tree: ElementTree = parse(mujoco_file)  # type: ignore

        _eoat_joint: list[EOATJoint] = []
        _eoat_joint.extend(
            EOATJoint(
                name=joint['name'],
                type=joint['type'],
                mimic=joint['mimic'],
                lower=joint['lower'],
                upper=joint['upper'],
            ) for joint in eoat['joints']
        )

        _eoat_pose: list[EOATPose] = []
        _eoat_pose.extend(
            EOATPose(
                name=name,
                pose=pose
            ) for name, pose in eoat['named_poses'].items()
        )

        _col_disabled: list[CollisionDisabled] = self._get_collision_disabled_pair(
            eoat)

        self._eoat.append(EOAT(
            model=eoat['name'],

            xacro_file=xacro_tree,
            xacro_macro=eoat['xacro']['macro'],
            mujoco_file=mujoco_tree,

            links=eoat['links'],
            base=eoat['base_link'],
            joints=_eoat_joint,
            poses=_eoat_pose,
            collision_disabled_pair=_col_disabled,
        ))

    def _generate_robot(self, model: StationModel) -> None:
        root: PosixPath = self._packages._asdict()[model.make]
        file: PosixPath = root / model.ros_descriptor

        with open(file, 'r') as data:
            robot = yaml.safe_load(data)

        xacro_file: PosixPath = root / robot['xacro']['file']
        xacro_tree: ElementTree = parse(xacro_file)  # type: ignore

        mujoco_root: PosixPath = self._packages.mujoco
        mujoco_file: PosixPath = mujoco_root / robot['mujoco']['file']
        mujoco_tree: ElementTree = parse(mujoco_file)  # type: ignore

        _robot_joint: list[RobotJoint] = []
        _robot_joint.extend(
            RobotJoint(
                name=joint['name'],
                max_velocity=joint['max_velocity'],
                max_acceleration=joint['max_acceleration'],
            ) for joint in robot['joints']
        )

        _robot_pose: list[RobotPose] = []
        _robot_pose.extend(
            RobotPose(
                name=name,
                pose=pose
            ) for name, pose in robot['named_poses'].items()
        )

        _col_disabled: list[CollisionDisabled] = self._get_collision_disabled_pair(
            robot)
        self._robots.append(
            Robot(
                model=robot['name'],

                xacro_file=xacro_tree,
                xacro_macro=robot['xacro']['macro'],
                mujoco_file=mujoco_tree,

                links=robot['links'],
                base=robot['base_link'],
                flange=robot['tip_link'],
                joints=_robot_joint,
                poses=_robot_pose,
                tool_adjacent=robot['tool_adjacent_links'],
                collision_disabled_pair=_col_disabled
            )
        )

    def _get_collision_disabled_pair(self, config: dict) -> list[CollisionDisabled]:
        result: list[CollisionDisabled] = []
        _col: dict[str, list] = config['collision']
        result.extend(
            CollisionDisabled(
                link_1=col['link1'], link_2=col['link2'], reason=col['reason']
            )
            for col in _col['disabled_pairs']
        )
        return result


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Generate assembly station files.')
    parser.add_argument('--station', type=Path, required=True)
    parser.add_argument('--dobot-share', type=Path, required=True)
    parser.add_argument('--eoat-share', type=Path, required=True)
    parser.add_argument('--mujoco-share', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args: argparse.Namespace = parser.parse_args()

    pkg = PackageShare(
        dobot=args.dobot_share.resolve(),
        eoat=args.eoat_share.resolve(),
        mujoco=args.mujoco_share.resolve(),
    )
    config: PosixPath = args.station.resolve()
    output_path = args.out.resolve()

    GenerateStation(config_file=config, packages=pkg, output_path=output_path)


if __name__ == '__main__':
    main()
