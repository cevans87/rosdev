from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.docker.container import Container
from rosdev.util.handler import Handler


log = getLogger(__package__)


@memoize
@dataclass(frozen=True)
class Bash(Handler):

    @memoize
    async def _main(self) -> None:
        await Container(self.options(command='/bin/bash', interactive=True))
