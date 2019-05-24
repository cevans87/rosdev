from atools import memoize
from dataclasses import dataclass
from frozendict import frozendict
from logging import getLogger
from typing import Mapping

from rosdev.gen.docker.container import Container
from rosdev.gen.rosdep.config import Config as RosdepConfig
from rosdev.util.handler import Handler


log = getLogger(__name__)


@memoize
@dataclass(frozen=True)
class Install(Handler):

    @property
    def command(self) -> str:
        if self.options.rosdep_install_args is None:
            return f'bash -c "rosdep update && rosdep install"'
        else:
            return f'bash -c "rosdep update && rosdep install {self.options.rosdep_install_args}"'

    @property
    def volumes(self) -> Mapping[str, str]:
        return frozendict({
            **self.options.volumes,
            **RosdepConfig(self.options).volumes,
        })

    async def _main(self) -> None:

        await RosdepConfig(self.options)

        await Container(
            options=self.options(
                command=self.command,
                volumes=self.volumes,
            )
        )
