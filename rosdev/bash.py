from dataclasses import dataclass, field
from logging import getLogger
import os
from typing import Tuple, Type

from rosdev.gen.docker.container import GenDockerContainer
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class Bash(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenDockerContainer,
    ))

    @classmethod
    async def main(cls, options: Options) -> None:
        log.info('Starting bash')
        os.execlpe(
            'docker',
            *(
                f'docker exec -it {options.docker_container_name} '
                f'{options.docker_entrypoint_sh_container_path} '
                f'/bin/bash'
            ).split(),
            os.environ
        )
