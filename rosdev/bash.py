from asyncio import gather
from atools import memoize
from dataclasses import dataclass, replace
from frozendict import frozendict
from logging import getLogger
from pathlib import Path
from typing import Mapping

from rosdev.gen.install import Install
from rosdev.gen.src import Src
from rosdev.gen.docker.container import Container
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@memoize
@dataclass(frozen=True)
class Bash(Handler):
    
    @property
    def command(self) -> str:
        return '/bin/bash'

    @property
    def options(self) -> Options:
        return replace(
            super().options,
            command=self.command,
            interactive=True,
            volumes=self.volumes,
        )

    @property
    def volumes(self) -> Mapping[str, str]:
        return frozendict({
            **super().options.volumes,
            f'{Path.home()}/.bashrc': f'{Path.home()}/.bashrc',
            f'{Path.home()}/.bash_history': f'{Path.home()}/.bash_history',
            f'{Path.home()}/.profile': f'{Path.home()}/.profile',
            **Install(super().options).options.volumes,
            **Src(super().options).options.volumes,
        })

    @memoize
    async def _main(self) -> None:
        await gather(
            Install(self.options),
            Src(self.options),
        )
        await Container(self.options)
