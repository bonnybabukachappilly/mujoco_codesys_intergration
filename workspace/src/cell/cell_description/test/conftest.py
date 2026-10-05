"""Shared fixtures for the generator tests."""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]              # cell_description
sys.path.insert(0, str(PKG / 'scripts'))

import generate_cell as gc  # noqa: E402


@pytest.fixture
def paths():
    """Locations of the real data files and of the test fixture."""
    src_root = PKG.parent.parent                       # workspace/src
    return SimpleNamespace(
        cell_share=PKG / 'test' / 'data' / 'cell_share',   # fixed 2-robot layout
        real_cell_share=PKG,                               # the real cell.yaml
        robot_share=src_root / 'robots' / 'dobot_cr5_description',
        mujoco_src=PKG.parent / 'cell_mujoco',         # config/cr5_sim.yaml
        urdf=PKG / 'test' / 'data' / 'cr5_expanded.urdf',
        script=PKG / 'scripts' / 'generate_cell.py',
    )


@pytest.fixture
def real_cell(paths):
    """Return ``(cell, registry)`` loaded from the REAL cell.yaml."""
    return gc.load_inputs(paths.real_cell_share)


@pytest.fixture
def urdf_text(paths):
    """The expanded CR5 URDF used as a fixture."""
    return paths.urdf.read_text()


@pytest.fixture
def robot(urdf_text):
    """The fixture URDF parsed by the generator."""
    return gc.parse_urdf(urdf_text)


@pytest.fixture
def cr5_data(paths):
    """Contents of dobot_cr5_description/config/cr5.yaml."""
    return yaml.safe_load((paths.robot_share / 'config/cr5.yaml').read_text())


@pytest.fixture
def generate(tmp_path, paths):
    """Return ``generate(target, **overrides)`` that runs the generator.

    Overrides: ``cell_share``, ``robot_share``, ``src``, ``urdf``. The output
    directory is returned.
    """
    def _generate(target, **overrides):
        args = {'cell_share': paths.cell_share,
                'robot_share': paths.robot_share,
                'src': paths.mujoco_src,
                'urdf': paths.urdf}
        args.update(overrides)
        out = tmp_path / target
        gc.main(target, args['cell_share'], out, args['src'],
                args['robot_share'], args['urdf'])
        return out
    return _generate


@pytest.fixture
def mujoco_out(generate):
    """Output directory of the ``mujoco`` target."""
    return generate('mujoco')


@pytest.fixture
def bare_robot(mujoco_out):
    """Root element of the robot MJCF without a tool."""
    return ET.parse(mujoco_out / 'robots/cr5/robot.xml').getroot()


@pytest.fixture
def tool_robot(mujoco_out):
    """Root element of the robot MJCF with the PGC gripper."""
    return ET.parse(mujoco_out / 'robots/cr5/robot_pgc_50_35.xml').getroot()


@pytest.fixture
def moveit_out(generate):
    """Output directory of the ``moveit`` target."""
    return generate('moveit')
