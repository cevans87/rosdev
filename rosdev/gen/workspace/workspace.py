from atools import memoize
from logging import getLogger
import os

from ..docker.container.container import container

log = getLogger(__name__)


@memoize
async def workspace(
        architecture: str,
        release: str,
        gdbserver_port: int,
) -> None:
    await container(
        architecture=architecture,
        release=release,
        interactive=False,
        ports=frozenset(),
        command=f'cp /opt/ros/{release} {os.getcwd()}/.rosdev/{architecture}/{release}',
    )
