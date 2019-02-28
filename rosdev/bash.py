from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import FrozenSet, Optional

from rosdev.gen.docker.container import Container
from rosdev.util.handler import Handler


log = getLogger(__package__)


@memoize
@dataclass(frozen=True)
class Bash(Handler):
    architecture: str
    build_num: Optional[int]
    fast: bool
    ports: FrozenSet[int]
    release: str

    @property
    @memoize
    def container(self) -> Container:
        return Container(
            architecture=self.architecture,
            build_num=self.build_num,
            command='/bin/bash',
            fast=self.fast,
            interactive=True,
            ports=self.ports,
            release=self.release,
        )

    @memoize
    async def _run(self) -> None:
        await self.container
