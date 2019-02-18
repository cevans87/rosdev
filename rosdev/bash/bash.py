from atools import memoize
from logging import getLogger
from typing import FrozenSet

from ..gen.docker.container.container import gen_docker_container


log = getLogger(__name__)


@memoize
async def bash(
        architecture: str,
        release: str,
        ports: FrozenSet[int],
) -> None:
    await gen_docker_container(
        architecture=architecture,
        release=release,
        ports=ports,
        interactive=True,
        command='/bin/bash'
    )
