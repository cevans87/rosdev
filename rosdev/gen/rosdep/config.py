from atools import memoize
from dataclasses import dataclass
from frozendict import frozendict
from logging import getLogger
from pathlib import Path
from typing import Mapping

from rosdev.gen.rosdev.config import Config as RosdevConfig
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.subprocess import exec


log = getLogger(__name__)


# FIXME We probably shouldn't need to store any config on the host. It should just be stored in a
#  persistent container. Basically, we should delete this.


@memoize
@dataclass(frozen=True)
class Config(Handler):

    @property
    def local_path(self) -> str:
        return f'{RosdevConfig(super().options).local_path}/ros'

    @property
    def container_path(self) -> str:
        return f'{Path.home()}/.ros'

    @property
    def options(self) -> Options:
        return super().options(
            volumes=frozendict({
                **super().options.volumes,
                self.local_path: self.container_path,
            }),
        )

    @property
    def volumes(self) -> Mapping[str, str]:
        # TODO
        return self._options.volumes

    async def _main(self) -> None:
        await exec(f'mkdir -p {self.local_path}')
