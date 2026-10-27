import logging
from pathlib import Path

from station_generator.emit.ros.srdf import SrdfEmitter
from station_generator.loaders.base import BaseLoader, get_loaders
from station_generator.model.base import parse_component
from station_generator.model.station import StationModel
from station_generator.state import ros_packages, station_components

log: logging.Logger = logging.getLogger("generator")


def parse_configuration(file: Path):
    log.debug(f'parsing file: {file}')
    loader: BaseLoader = get_loaders(file.suffix)
    data: object = loader.load(file)

    try:
        # Create Station Model
        station: StationModel = StationModel.model_validate(data)

        # Create Component models
        for key, component in station.components.items():

            _package: str = component.package
            _file: Path = ros_packages.fetch(_package) / component.descriptor
            _loader: BaseLoader = get_loaders(_file.suffix)
            _data: object = _loader.load(_file)

            station_components.register(
                key,
                parse_component(_data)  # type: ignore
            )

        s = SrdfEmitter()
        s.emit(station)

    except ValueError as e:
        log.error(e)
