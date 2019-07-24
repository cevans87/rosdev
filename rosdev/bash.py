from dataclasses import dataclass, field, replace
from logging import getLogger
from typing import Tuple, Type

from rosdev.gen.docker.image import GenDockerImage
from rosdev.gen.ros.install import GenRosInstall
from rosdev.gen.ros.src import GenRosSrc
from rosdev.gen.docker.container import GenDockerContainer
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class Bash(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenDockerImage,
        GenRosInstall,
        GenRosSrc,
    ))
    post_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenDockerContainer,
    ))

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        return replace(
            options,
            docker_container_command='/bin/bash',
            interactive_docker_container=True,
        )
