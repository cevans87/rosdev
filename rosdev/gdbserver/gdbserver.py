from atools import memoize
from logging import getLogger

from ..gen.docker.container.container import container

log = getLogger(__name__)


@memoize
async def gdbserver(
        architecture: str,
        release: str,
        gdbserver_port: int,
        package: str,
        executable: str,
) -> None:
    await container(
        architecture=architecture,
        release=release,
        interactive=True,
        ports=frozenset({gdbserver_port}),
        command=f'ros2 run --prefix "gdbserver :{gdbserver_port}" {package} {executable}',
    )
