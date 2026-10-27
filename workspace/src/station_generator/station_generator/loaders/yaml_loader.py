from pathlib import Path
from typing import ClassVar

from yaml import YAMLError, safe_load

from station_generator.errors import ConfigError

from .base import register_loaders


@register_loaders
class YAMLLoader:
    name: ClassVar[str] = '.yaml'

    def load(self, path: Path) -> object:
        try:
            file: str = path.read_text(encoding='utf-8')
            return safe_load(file)
        except FileNotFoundError as e:
            raise ConfigError(f"{path}: file not found") from e
        except OSError as e:
            raise ConfigError(f"{path}: cannot read ({e.strerror})") from e
        except YAMLError as e:
            raise ConfigError(f"{path}: invalid YAML: {e}") from e
