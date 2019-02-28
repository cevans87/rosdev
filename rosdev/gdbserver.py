from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.docker.container import Container
from rosdev.util.handler import Handler

log = getLogger(__package__)


@memoize
@dataclass(frozen=True)
class Gdbserver(Handler):
    architecture: str
    build_num: int
    fast: bool
    executable: str
    package: str
    port: int
    release: str

    @property
    def container(self) -> Container:
        return Container(
            architecture=self.architecture,
            build_num=self.build_num,
            command=f'ros2 run --prefix "gdbserver :{self.port}" {self.package} {self.executable}',
            fast=self.fast,
            interactive=True,
            ports=frozenset({self.port}),
            release=self.release,
        )

    @memoize
    async def _run(self) -> None:
        await self.container
