from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.docker.container import GenDockerContainer
from rosdev.gen.docker.pam_environment import GenDockerPamEnvironment
from rosdev.gen.docker.ssh_base import GenDockerSshBase
from rosdev.gen.host import GenHost
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerSsh(GenDockerSshBase):
    
    @staticmethod
    @memoize
    async def get_port(options: Options) -> int:
        port = int(
            (
                await GenHost.execute_and_get_line(
                    command=f'docker port {await GenDockerContainer.get_name(options)} 22',
                    options=options,
                )
            ).rsplit(':', 1)[-1]
        )

        log.debug(f'{GenDockerSsh.__name__} {port = }')

        return port

    @classmethod
    async def main(cls, options: Options) -> None:
        assert (await GenDockerPamEnvironment.get_path(options)).is_file()
        await GenHost.execute(
            command=(
                f'ssh-keygen'
                f' -f "{await GenDockerSshBase.get_path(options) / "known_hosts"}"'
                f' -R "[localhost]:{await cls.get_port(options)}"'
            ),
            err_ok=True,
            options=options,
        )

        await GenDockerContainer.execute(
            command=f'sudo service ssh start',
            options=options,
        )
        await GenHost.execute_shell(
            command=(
                f'ssh-keyscan -p {await cls.get_port(options)} localhost >>'
                f' "{await GenDockerSshBase.get_path(options) / "known_hosts"}"'
            ),
            options=options,
        )
