from __future__ import annotations

import argparse
import logging
from collections.abc import Sequence
from pathlib import Path

from station_generator.resolvers import parse_configuration
from station_generator.state import ros_packages

log: logging.Logger = logging.getLogger("generator")


def write_manifest(out_dir: Path, generated_files: list[Path]) -> None:
    manifest_path: Path = out_dir / "manifest.txt"
    out_dir.mkdir(parents=True, exist_ok=True)

    relative_paths: list[str] = [
        str(p.relative_to(out_dir)) for p in generated_files]
    manifest_path.write_text("\n".join(sorted(relative_paths)) + "\n")


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="station_generator",
        description="Generate assembly station files.",
    )

    # required inputs
    p.add_argument(
        "--station", type=Path, required=True,
        help="station_configuration.yaml")
    p.add_argument(
        "--out", type=Path, required=True,
        help="output directory")
    p.add_argument("--dobot-share", type=Path, required=True)
    p.add_argument("--eoat-share", type=Path, required=True)
    p.add_argument("--mujoco-share", type=Path, required=True)

    # optional behavior
    p.add_argument("--ros-distro", default="humble")
    p.add_argument(
        "--only", nargs="*", metavar="EMITTER",
        help="run only these emitters (plus nothing else)")
    p.add_argument(
        "--list", action="store_true",
        help="list registered emitters and exit")
    p.add_argument(
        "--dry-run", action="store_true",
        help="compute everything, write nothing")
    p.add_argument("-v", "--verbose", action="store_true")
    return p


def _setup_logging(verbose: bool) -> None:
    logging.basicConfig(
        level=logging.DEBUG if verbose else logging.WARNING,
        format="%(message)s",
    )


def main(argv: Sequence[str] | None = None) -> int:
    args: argparse.Namespace = _build_parser().parse_args(argv)
    _setup_logging(args.verbose)

    ros_packages.register('dobot_description', args.dobot_share)
    ros_packages.register('eoat_description', args.eoat_share)
    ros_packages.register('mujoco_system', args.mujoco_share)

    log.info('********************************************************')
    parse_configuration(args.station)
    log.info('********************************************************')

    write_manifest(args.out, [])

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
