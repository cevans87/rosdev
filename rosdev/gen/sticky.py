from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import FrozenSet, Optional

from rosdev.util.handler import Handler


log = getLogger(__package__)


@memoize
@dataclass(frozen=True)
class Sticky(Handler):
    architecture: str
    build_num: Optional[int]
    clean: bool
    fast: bool
    ports: FrozenSet[int]
    release: str

    @memoize
    async def _main(self) -> None:
        await self.container
