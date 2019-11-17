from dataclasses import dataclass
from logging import getLogger
import os
from pathlib import Path

from rosdev.gen.docker.container import GenDockerContainer
from rosdev.gen.docker.entrypoint_sh import GenDockerEntrypointSh
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class Bash(Handler):

    @classmethod
    async def main(cls, options: Options) -> None:
        log.info('Starting bash.')
        os.execlpe(
            'docker',
            *(
                f'docker exec -it'
                f' -e {await GenDockerEntrypointSh.get_log_level_env_name(options)}'
                f'={log.getEffectiveLevel()}'
                f' {await GenDockerEntrypointSh.get_environment_flags(options)}'
                f' --workdir {Path.cwd()}'
                f' {await GenDockerContainer.get_name(options)}'
                f' {await GenDockerEntrypointSh.get_container_path(options)}'
                f' /bin/bash'
            ).split(),
            os.environ
        )
