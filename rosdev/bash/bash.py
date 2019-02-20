from atools import memoize
from logging import getLogger
from typing import FrozenSet

from ..gen.docker.container.container import container


log = getLogger(__name__)


@memoize
async def bash(
        architecture: str,
        nightly: bool,
        release: str,
        ports: FrozenSet[int],
) -> None:
    await container(
        architecture=architecture,
        nightly=nightly,
        release=release,
        ports=ports,
        interactive=True,
        command='/bin/bash'
    )
