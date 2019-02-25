from atools import memoize
from logging import getLogger

from rosdev.gen.docker.container import container

log = getLogger(__package__)


@memoize
async def gdbserver(
        architecture: str,
        build_num: int,
        executable: str,
        package: str,
        port: int,
        release: str,
) -> None:
    await container(
        architecture=architecture,
        build_num=build_num,
        command=f'ros2 run --prefix "gdbserver :{port}" {package} {executable}',
        interactive=True,
        ports=frozenset({port}),
        release=release,
    )
