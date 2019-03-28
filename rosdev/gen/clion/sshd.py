from atools import memoize
from dataclasses import dataclass
from pathlib import Path

from rosdev.gen.docker.container import Container
from rosdev.util.handler import Handler
from rosdev.util.subprocess import exec


@memoize
@dataclass(frozen=True)
class Sshd(Handler):

    @memoize
    async def _main(self) -> None:
        await exec(f'ssh-keygen -f "{Path.home()}/.ssh/known_hosts" -R "localhost"')
        await Container(
            options=self.options(
                command='bash -c "sudo service ssh start && sleep infinity"',
                ports=frozenset({22}),
                name='rosdev_clion_sshd',
            ),
        )
