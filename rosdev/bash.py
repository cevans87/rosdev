from atools import memoize
from logging import getLogger
from typing import FrozenSet, Optional

from rosdev.gen.docker.container import container


log = getLogger(__package__)


@memoize
async def bash(
        *,
        architecture: str,
        build_num: Optional[int],
        ports: FrozenSet[int],
        release: str,
) -> None:
    await container(
        architecture=architecture,
        build_num=build_num,
        command='/bin/bash',
        interactive=True,
        ports=ports,
        release=release,
    )
