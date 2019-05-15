from atools import memoize
from dataclasses import dataclass
from frozendict import frozendict
from logging import getLogger
from pathlib import Path

from rosdev.gen.config import Config
from rosdev.util.handler import Handler
from rosdev.util.subprocess import exec


log = getLogger(__name__)


@memoize
@dataclass(frozen=True)
class Rosdep(Handler):

    @property
    def local_path(self) -> str:
        return f'{Config(self.options).local_path}/ros'

    @property
    def container_path(self) -> str:
        return f'{Path.home()}/.ros'

    @property
    def volumes(self) -> frozendict:
        return frozendict({
            **self.options.volumes,
            self.local_path: self.container_path,
        })

    async def _main(self) -> None:
        await exec(f'mkdir -p {self.local_path}')
