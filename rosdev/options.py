from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
import logging
from typing import FrozenSet

from rosdev.path import Path

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class Options:
    class Architecture(str, Enum):
        amd64 = 'amd64'
        arm32v7 = 'arm32v7'
        arm64v8 = 'arm64v8'

    class LogLevel(str, Enum):
        debug = 'debug'
        info = 'info'
        warning = 'warning'
        fatal = 'fatal'

    @dataclass(frozen=True)
    class Mount:
        container_path: Path
        host_path: Path

    @dataclass(frozen=True)
    class Port:
        container: int
        host: int

    class Release(str, Enum):
        # ROS
        kinetic = 'kinetic'
        melodic = 'melodic'
        noetic = 'noetic'

        # ROS2
        dashing = 'dashing'
        foxy = 'foxy'

    architecture: Architecture
    args: str
    log_level: LogLevel
    mounts: FrozenSet[Mount]
    ports: FrozenSet[Port]
    release: str
