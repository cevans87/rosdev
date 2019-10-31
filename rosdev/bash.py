from dataclasses import dataclass, field
from logging import getLogger
import os
from pathlib import Path
from typing import Tuple, Type

from rosdev.gen.docker.core import GenDockerCore
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class Bash(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenDockerCore,
    ))

    @classmethod
    async def main(cls, options: Options) -> None:
        log.info('Starting bash.')
        os.execlpe(
            'docker',
            *(
                f'docker exec -it '
                f'{options.docker_environment_flags} '
                f'--workdir {Path.cwd()} '
                f'{options.docker_container_name} '
                f'{options.docker_entrypoint_sh_container_path} '
                f'/bin/bash'
            ).split(),
            os.environ
        )
