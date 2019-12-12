from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.runner import GenBackendRunner
from rosdev.gen.backend.entrypoint_sh.runner_base import GenBackendEntrypointhRunnerBase
from rosdev.gen.backend.workspace.runner_base import GenBackendWorkspaceRunnerBase
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class Run(Handler):
    """Run rosrun/ros2 run on runner endpoint."""

    @staticmethod
    @memoize
    async def main(options: Options) -> None:
        await GenBackendRunner.get_ssh(options).execute(
            command=(
                f'{await GenBackendEntrypointhRunnerBase.get_path(options)}'
                f' {"rosrun" if options.release in {"kinetic", "melodic"} else "ros2 run"}'
                f'{" " + " ".join(options.remainder) if options.remainder else ""}'
            ),
            environment=await GenBackendEntrypointhRunnerBase.get_environment(options),
            options=options,
            path=await GenBackendWorkspaceRunnerBase.get_path(options),
        )
