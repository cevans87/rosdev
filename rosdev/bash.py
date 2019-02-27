from __future__ import annotations
from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import Generator, FrozenSet, Optional

from rosdev.gen.docker.container import container


log = getLogger(__package__)


@dataclass(frozen=True)
class _Bash:
    architecture: str
    build_num: Optional[int]
    ports: FrozenSet[int]
    release: str

    def __await__(self) -> Generator[_Bash, None, None]:
        return self._run().__await__()

    @memoize
    async def _run(self) -> None:
        await container(
            architecture=self.architecture,
            build_num=self.build_num,
            command='/bin/bash',
            interactive=True,
            ports=self.ports,
            release=self.release,
        )


bash = memoize(_Bash)
