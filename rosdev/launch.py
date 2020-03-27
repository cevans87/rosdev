from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.entrypoint_sh.runner import GenBackendEntrypointShRunner
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class Launch(Handler):
    """Run roslaunch/ros2 launch on runner endpoint."""

    @staticmethod
    @memoize
    async def main(options: Options) -> None:
        await GenBackendEntrypointShRunner.execute(
            command=(
                f'{"roslaunch" if options.release in {"kinetic", "melodic"} else "ros2 launch"}'
                f'{" " + options.remainder if options.remainder else ""}'
            ),
            options=options,
        )
