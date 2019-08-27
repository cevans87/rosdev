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
        await cls.exec_workspace(
            options=options,
            command=f'ssh-keygen -f "{Path.home()}/.ssh/known_hosts" -R "localhost"',
            err_ok=True
        )
        await cls.exec_container(options=options, command='sudo service ssh start')
