from atools import memoize
from dataclasses import dataclass
from frozendict import frozendict
from logging import getLogger
from pathlib import Path

from rosdev.gen.rosdev.config import Config as RosdevConfig
from rosdev.util.handler import Handler
from rosdev.util.subprocess import exec


log = getLogger(__name__)


@memoize
@dataclass(frozen=True)
class Config(Handler):

    @property
    def container_path(self) -> str:
        return f'{Path.home()}/.colcon'

    @property
    def local_path(self) -> str:
        return f'{RosdevConfig(self.options).local_path}/colcon'

    @property
    def volumes(self) -> frozendict:
        return frozendict({
            **self.options.volumes,
            self.global_path: self.global_path,
            self.local_path: self.container_path
        })

    @memoize
    async def _main(self) -> None:
        await exec(f'mkdir -P {self.local_path}')
