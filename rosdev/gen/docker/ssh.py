from dataclasses import dataclass, field
from logging import getLogger
from pathlib import Path
from typing import Tuple, Type

from rosdev.gen.docker.container import GenDockerContainer
from rosdev.gen.docker.pam_environment import GenDockerPamEnvironment
from rosdev.gen.host import GenHost
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerSsh(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenDockerContainer,
        GenDockerPamEnvironment,
        GenHost,
    ))

    @classmethod
    async def main(cls, options: Options) -> None:
        await GenHost.execute(
            command=(
                f'ssh-keygen '
                f'-f "{Path.home()}/.ssh/known_hosts" '
                f'-R "[localhost]:{await GenDockerContainer.get_ssh_port(options)}"'
            ),
            err_ok=True,
            options=options,
        )
        await GenHost.execute(
            command=f'docker exec {options.docker_container_name} sudo service ssh start',
            options=options,
        )
        await GenHost.execute_shell(
            command=(
                f'ssh-keyscan -p {await GenDockerContainer.get_ssh_port(options)} '
                f'localhost >> "{Path.home()}/.ssh/known_hosts"'
            ),
            options=options,
        )
