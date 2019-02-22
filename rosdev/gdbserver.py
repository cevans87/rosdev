from atools import memoize
from logging import getLogger

from rosdev.gen.docker.container import container

log = getLogger(__package__)


@memoize
async def gdbserver(
        architecture: str,
        release: str,
        port: int,
        package: str,
        executable: str,
) -> None:
    await container(
        architecture=architecture,
        release=release,
        interactive=True,
        ports=frozenset({port}),
        command=f'ros2 run --prefix "gdbserver :{port}" {package} {executable}',
    )
