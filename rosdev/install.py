from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.endpoints import GenBackendEndpoints
from rosdev.gen.backend.sync.apt.packages import GenBackendSyncAptPackages

from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class Install(Handler):
    """Run rosdep install on all endpoints."""

    @staticmethod
    @memoize
    async def main(options: Options) -> None:
        await GenBackendEndpoints.execute_by_endpoint(
            command=(
                f'rosdep install'
                f'{" " + " ".join(options.remainder) if options.remainder else ""}'
            ),
            options=options,
        )
