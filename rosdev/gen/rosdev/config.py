from asyncio import gather
from atools import memoize
from dataclasses import dataclass
from frozendict import frozendict
from logging import getLogger
from pathlib import Path
from typing import Mapping

from rosdev.util.build_farm import get_build_num
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
    async def get_build_num(self) -> int:
        build_num = self.options.build_num
        if (build_num is None) and \
                (not self.options.pull_install) and \
                (not self.options.pull_src):
            try:
                paths = sorted(Path(self.global_path).iterdir())
            except FileNotFoundError:
                pass
            else:
                try:
                    build_num = int(str(paths[-1].parts[-1]))
                except (IndexError, ValueError):
                    pass

        if build_num is None:
            build_num = await get_build_num(
                architecture=self.options.architecture, release=self.options.release
            )

        return build_num

    @memoize
    async def _main(self) -> None:
        await gather(
            exec(f'mkdir -p {self.global_path}'),
            exec(f'mkdir -p {self.local_path}')
        )
