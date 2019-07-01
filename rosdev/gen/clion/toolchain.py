from asyncio import gather
from atools import memoize
from dataclasses import dataclass
from frozendict import frozendict
from logging import getLogger
from pathlib import Path
from typing import Mapping

from rosdev.gen.clion.cmake import Cmake

from rosdev.gen.gdbinit import Gdbinit
from rosdev.gen.docker.container import Container
from rosdev.gen.install import Install
from rosdev.gen.src import Src
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.subprocess import exec


log = getLogger(__name__)


@memoize
@dataclass(frozen=True)
class Toolchain(Handler):

    @property
    def options(self) -> Options:
        return super().options(
            command='bash -c "sudo service ssh start && sleep infinity"',
            container_name='rosdev_gen_clion_toolchain',
            ports=frozenset({22}),
            volumes=frozendict({
                **super().options.volumes,
                **Cmake(super().options).options.volumes,
                **Gdbinit(super().options).options.volumes,
                f'{Path.home()}/.ssh': f'{Path.home()}/.ssh',
            })
        )

    @memoize
    async def _main(self) -> None:
        await gather(
            Cmake(self.options),
            Gdbinit(self.options),
            Install(self.options),
            Src(self.options),
        )
        await exec(f'ssh-keygen -f "{Path.home()}/.ssh/known_hosts" -R "localhost"', err_ok=True)
        await Container(self.options)

        log.info('Toolchain running at localhost:22')
