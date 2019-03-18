from atools import memoize
from logging import getLogger

from rosdev.gen.docker.container import Container
from rosdev.util.handler import Handler

log = getLogger(__package__)


@memoize
class Gdbserver(Handler):

    @property
    def container(self) -> Container:
        return Container(
            clean=False,
            command=(
                f'ros2 run --prefix "gdbserver :{self.gdbserver_port}" '
                f'{self.package} {self.executable}'
            ),
            interactive=True,
            ports=frozenset({self.gdbserver_port}),
        )

    @memoize
    async def _main(self) -> None:
        await self.container
