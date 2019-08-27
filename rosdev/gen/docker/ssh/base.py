from dataclasses import dataclass, field
from logging import getLogger
from typing import Tuple, Type

from rosdev.gen.docker.ssh.port import GenDockerSshPort
from rosdev.util.handler import Handler


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerSshBase(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenDockerSshPort,
    ))
