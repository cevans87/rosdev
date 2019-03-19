from atools import memoize

from rosdev.gen.docker.container import Container
from rosdev.util.handler import Handler


@memoize
class Gazebo(Handler):

    @memoize
    async def _main(self) -> None:
        await Container(
            self.options(
                command='gazebo',
                flavor='desktop-full',
                gui=True,
                interactive=True,
            )
        )
