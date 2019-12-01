from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.runner_endpoint import GenBackendRunnerEndpoint
from rosdev.gen.backend.sync.apt.packages import GenBackendSyncAptPackages

from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class Launch(Handler):
    """Run roslaunch/ros2 launch on runner endpoint."""

    @staticmethod
    @memoize
    async def main(options: Options) -> None:
        await GenBackendRunnerEndpoint.execute(
            command=(
                f'{"roslaunch" if options.release in {"kinetic", "melodic"} else "ros2 launch"}'
                f'{" " + " ".join(options.remainder) if options.remainder else ""}'
            ),
            options=options,
        )
