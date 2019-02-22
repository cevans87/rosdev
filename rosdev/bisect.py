from atools import memoize
from logging import getLogger
from typing import FrozenSet, Optional, Tuple

from rosdev.gen.docker.container import container


log = getLogger(__package__)


@memoize
def builds_between(good: int, bad: int) -> Tuple[int]:
    pass


@memoize
async def bisect(
        *,
        architecture: str,
        colcon_args: Optional[str],
        ports: FrozenSet[int],
        release: str,
) -> None:
    await container(
        architecture=architecture,
        release=release,
        ports=ports,
        interactive=False,
        command=f'colcon build{" " + " ".join(colcon_args) if colcon_args else ""}'
    )


handler = bisect
