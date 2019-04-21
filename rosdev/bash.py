from atools import memoize
from dataclasses import dataclass
from frozendict import frozendict
from logging import getLogger
from pathlib import Path

from rosdev.gen.docker.container import Container
from rosdev.util.handler import Handler


log = getLogger(__name__)


@memoize
@dataclass(frozen=True)
class Bash(Handler):

    @property
    def volumes(self) -> frozendict:
        return frozendict({
            **self.options.volumes,
            f'{Path.home()}/.bashrc': f'{Path.home()}/.bashrc',
            f'{Path.home()}/.bash_history': f'{Path.home()}/.bash_history',
            f'{Path.home()}/.profile': f'{Path.home()}/.profile',
        })

    @memoize
    async def _main(self) -> None:
        await Container(
            self.options(
                command='/bin/bash',
                interactive=True,
                volumes=self.volumes,
            )
        )
