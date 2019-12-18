from asyncio import gather
from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.packages.mixin_base import GenBackendAptPackagesMixinBase
from rosdev.gen.backend.builder import GenBackendBuilder
from rosdev.gen.backend.entrypoint_sh.builder_base import GenBackendEntrypointhBuilderBase
from rosdev.gen.backend.entrypoint_sh.local_base import GenBackendEntrypointhLocalBase
from rosdev.gen.backend.entrypoint_sh.runner_base import GenBackendEntrypointhRunnerBase
from rosdev.gen.backend.local import GenBackendLocal
from rosdev.gen.backend.pip.packages.mixin_base import GenBackendPipPackagesMixinBase
from rosdev.gen.backend.runner import GenBackendRunner
from rosdev.gen.backend.workspace.builder_base import GenBackendWorkspaceBuilderBase
from rosdev.gen.backend.workspace.local_base import GenBackendWorkspaceLocalBase
from rosdev.gen.backend.workspace.runner_base import GenBackendWorkspaceRunnerBase
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class RosdepInstall(Handler):
    """Run apt install on all endpoints."""

    @staticmethod
    @memoize
    async def main(options: Options) -> None:
        await gather(
            GenBackendAptPackagesMixinBase.get_apt_packages.memoize.reset(),
            GenBackendPipPackagesMixinBase.get_pip_packages.memoize.reset(),
        )

        await gather(
            GenBackendBuilder.get_ssh(options).execute(
                command=(
                    f'apt install {" " + " ".join(options.remainder) if options.remainder else ""}'
                ),
                environment=await GenBackendEntrypointhBuilderBase.get_environment(options),
                options=options,
                path=await GenBackendWorkspaceBuilderBase.get_path(options),
                sudo=True,
            ),
            GenBackendLocal.get_ssh(options).execute(
                command=(
                    f'apt install {" " + " ".join(options.remainder) if options.remainder else ""}'
                ),
                environment=await GenBackendEntrypointhLocalBase.get_environment(options),
                options=options,
                path=await GenBackendWorkspaceLocalBase.get_path(options),
                sudo=True,
            ),
            GenBackendRunner.get_ssh(options).execute(
                command=(
                    f'apt install {" " + " ".join(options.remainder) if options.remainder else ""}'
                ),
                environment=await GenBackendEntrypointhRunnerBase.get_environment(options),
                options=options,
                path=await GenBackendWorkspaceRunnerBase.get_path(options),
                sudo=True,
            ),
        )
