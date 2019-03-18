from atools import memoize
from dataclasses import dataclass

from rosdev.gen.docker.container import Container
from rosdev.util.handler import Handler


@memoize
@dataclass(frozen=True)
class Gazebo(Handler):

    @memoize
    async def _main(self) -> None:
        await Container(
            self.options(
                command='gazebo --verbose',
                flavor='desktop-full',
                gui=True,
                interactive=True,
            )
        )
