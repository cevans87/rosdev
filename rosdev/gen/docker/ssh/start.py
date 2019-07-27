from dataclasses import dataclass, field, replace
from logging import getLogger
from pathlib import Path
from typing import Tuple, Type

from rosdev.gen.docker.container import GenDockerContainer
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.subprocess import exec


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerSshStart(Handler):

    post_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenDockerContainer,
    ))

    @classmethod
    async def main(cls, options: Options) -> None:
        await exec(f'ssh-keygen -f "{Path.home()}/.ssh/known_hosts" -R "localhost"', err_ok=True)
        await exec(f'docker exec {options.docker_container_name} sudo service ssh start')
