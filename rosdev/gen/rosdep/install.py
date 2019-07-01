from atools import memoize
from dataclasses import dataclass
from frozendict import frozendict
from logging import getLogger
from pathlib import Path
from stringcase import snakecase
from typing import Mapping

from rosdev.gen.docker.container import Container
from rosdev.gen.rosdep.config import Config as RosdepConfig
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@memoize
@dataclass(frozen=True)
class Install(Handler):

    @property
    def command(self) -> str:
        if super().options.rosdep_install_args is None:
            return f'bash -c "rosdep update && rosdep install"'
        return f'bash -c "rosdep update && rosdep install {super().options.rosdep_install_args}"'

    @property
    def container_name(self) -> str:
        return (
                f'rosdev_gen_rosdep_install_at_'
                f'{snakecase(str(Path.cwd().relative_to(Path.home())).replace("/", "_"))}'
        )

    @property
    def options(self) -> Options:
        return super().options(
            command=self.command,
            container_name=self.container_name,
            volumes=self.volumes,
        )

    @property
    def volumes(self) -> Mapping[str, str]:
        return frozendict({
            **super().options.volumes,
            **RosdepConfig(super().options).volumes,
        })

    async def _main(self) -> None:
        await RosdepConfig(self.options)
        await Container(self.options)
