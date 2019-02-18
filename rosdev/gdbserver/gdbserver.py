from atools import memoize
from logging import getLogger
from typing import Tuple

from ..gen.docker.container.container import gen_docker_container

log = getLogger(__name__)


@memoize
async def gdbserver(
        architecture: str,
        release: str,
        port: int,
        binary: str,
        remainder: Tuple[str],
) -> None:
    await gen_docker_container(
        architecture=architecture,
        release=release,
        interactive=True,
        ports=frozenset({port}),
        command=(
            f'gdbserver :{port} '
            f'install/{binary}/lib/{binary}/{binary}'
            f'{" --args " + " ".join(remainder) if remainder else ""}'
        ),
    )
