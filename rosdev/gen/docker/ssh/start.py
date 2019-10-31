from dataclasses import dataclass, field
from logging import getLogger
from pathlib import Path
from typing import Tuple, Type

from rosdev.gen.docker.core import GenDockerCore
from rosdev.gen.docker.ssh.pam_environment import GenDockerSshPamEnvironment
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerSshStart(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenDockerCore,
        GenDockerSshPamEnvironment,
    ))

    @classmethod
    async def main(cls, options: Options) -> None:
        await cls.execute_host(
            command=(
                f'ssh-keygen '
                f'-f "{Path.home()}/.ssh/known_hosts" '
                f'-R "[localhost]:{options.docker_ssh_port}"'
            ),
            err_ok=True,
        )
        await cls.execute_shell_host(
            command=(
                f'ssh-keyscan '
                f'-p {options.docker_ssh_port} '
                f'localhost '
                f'>> "{Path.home()}/.ssh/known_hosts"'
            ),
        )
        await cls.execute_host(
            command=f'docker exec {options.docker_container_name} sudo service ssh start'
        )
