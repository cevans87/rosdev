from dataclasses import dataclass, field
from logging import getLogger
from typing import Tuple, Type

from rosdev.gen.docker.ssh.base import GenDockerSshBase
from rosdev.gen.docker.ssh.start import GenDockerSshStart
from rosdev.util.handler import Handler

log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerSshCore(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenDockerSshBase,
        GenDockerSshStart,
    ))
