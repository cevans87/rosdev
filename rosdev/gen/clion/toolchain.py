from asyncio import gather
from atools import memoize
from dataclasses import dataclass
from frozendict import frozendict
from logging import getLogger
from pathlib import Path

from rosdev.gen.clion.cmake import Cmake
from rosdev.gen.clion.gdb import Gdb
from rosdev.gen.docker.container import Container
from rosdev.gen.install import Install
from rosdev.gen.src import Src
from rosdev.util.handler import Handler
from rosdev.util.subprocess import exec


log = getLogger(__name__)


@memoize
@dataclass(frozen=True)
class Toolchain(Handler):

    @property
    def volumes(self) -> frozendict:
        return frozendict({
            **self.options.volumes,
            **Cmake(self.options).volumes,
            **Gdb(self.options).volumes,
            f'{Path.home()}/.ssh': f'{Path.home()}/.ssh',
        })

    @memoize
    async def _main(self) -> None:
        await gather(
            Cmake(self.options),
            Gdb(self.options),
            Install(self.options),
            Src(self.options),
        )
        await exec(f'ssh-keygen -f "{Path.home()}/.ssh/known_hosts" -R "localhost"', err_ok=True)
        await Container(
            options=self.options(
                command='bash -c "sudo service ssh start && sleep infinity"',
                ports=frozenset({22}),
                name='rosdev_gen_clion_toolchain',
                volumes=self.volumes,
            ),
        )

        log.info('Toolchain running at localhost:22')
