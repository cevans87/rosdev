from dataclasses import dataclass, field
from logging import getLogger
from typing import Tuple, Type

from rosdev.gen.base import GenBase
from rosdev.gen.docker.dockerfile import GenDockerDockerfile
from rosdev.gen.docker.entrypoint_sh import GenDockerEntrypointSh
from rosdev.util.handler import Handler


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerBase(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenBase,
        GenDockerDockerfile,
        GenDockerEntrypointSh,
    ))
