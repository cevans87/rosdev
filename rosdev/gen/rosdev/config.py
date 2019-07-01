from asyncio import gather
from atools import memoize
from dataclasses import dataclass
from frozendict import frozendict
from logging import getLogger
from pathlib import Path
from typing import Mapping

from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.subprocess import exec


log = getLogger(__name__)


@memoize
@dataclass(frozen=True)
class Config(Handler):

    @property
    def container_path(self) -> str:
        return f'{Path.cwd()}/.rosdev'

    @property
    def global_path(self) -> str:
        return f'{Path.home()}/.rosdev'

    @property
    def local_path(self) -> str:
        return f'{Path.cwd()}/.rosdev'

    @property
    def options(self) -> Options:
        return super().options(
            volumes=self.volumes,
        )

    @property
    def volumes(self) -> Mapping[str, str]:
        return frozendict({
            **super().options.volumes,
            self.global_path: self.global_path,
            self.local_path: self.container_path
        })

    @memoize
    async def _main(self) -> None:
        await gather(
            exec(f'mkdir -p {self.global_path}'),
            exec(f'mkdir -p {self.local_path}')
        )
