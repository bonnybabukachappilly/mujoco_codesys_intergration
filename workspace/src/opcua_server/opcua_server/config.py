from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class ServerConfig:
    endpoint: str
    namespace_uri: str
    server_name: str
    publish_rate: float


@dataclass(frozen=True)
class BridgeConfig:
    server: ServerConfig
    modules: dict[str, dict[str, Any]]


def load_config(path: str | Path) -> BridgeConfig:
    raw = yaml.safe_load(Path(path).read_text()) or {}
    modules = {
        name: (cfg or {})
        for name, cfg in (raw.get('modules') or {}).items()
        if (cfg or {}).get('enabled', True)
    }
    return BridgeConfig(ServerConfig(**(raw.get('server') or {})), modules)
