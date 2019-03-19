from atools import memoize
from logging import getLogger

from rosdev.gen.docker.container import Container
from rosdev.util.handler import Handler


log = getLogger(__name__)


@memoize
class Bash(Handler):

    @memoize
    async def _main(self) -> None:
        await Container(self.options(command='/bin/bash', interactive=True))
