import asyncio
from atools import memoize
import docker
from itertools import product
from logging import getLogger
import os
from pathlib import Path
from typing import FrozenSet, List

from ..gen.docker.container.container import gen_docker_container

log = getLogger(__name__)


@memoize
async def gdb(
        architecture: str,
        release: str,
        interactive: bool,
        port: int,
        remainder: str
) -> None:
    await gen_docker_container(
        architecture=architecture,
        release=release,
        interactive=interactive,
        ports=frozenset({port}),
        command=f'gdbserver --once localhost:{port} {remainder}'
    )
    await asyncio.gather(
        *[gen_docker_container(architecture, release, interactive, frozenset({port}))
          for architecture, release in product(architecture, release)])
