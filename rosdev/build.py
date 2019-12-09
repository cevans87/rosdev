from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.builder import GenBackendBuilder
from rosdev.gen.backend.entrypoint_sh.builder_base import GenBackendEntrypointhBuilderBase
from rosdev.gen.backend.local import GenBackendLocal
from rosdev.gen.backend.rsync.workspace.builder import GenBackendRsyncWorkspaceBuilder
from rosdev.gen.backend.workspace.builder_base import GenBackendWorkspaceBuilderBase
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class Build(Handler):
    """Run colcon build on build endpoint."""

    @staticmethod
    @memoize
    async def main(options: Options) -> None:
        await (await GenBackendBuilder.get_ssh(options)).execute(
            command=(
                f'{await GenBackendEntrypointhBuilderBase.get_path(options)}'
                f' colcon build'
                f'{" " + " ".join(options.remainder) if options.remainder else ""}'
            ),
            environment=await GenBackendEntrypointhBuilderBase.get_environment(options),
            options=options,
            path=await GenBackendWorkspaceBuilderBase.get_path(options),
        )

        if not await GenBackendRsyncWorkspaceBuilder.is_local(options):
            dst_uri = await GenBackendRsyncWorkspaceBuilder.get_dst_uri(options)
            f' {dst_uri.username}@{dst_uri.hostname}{dst_uri.path}'
            await (await GenBackendLocal.get_ssh(options)).execute(
                command=(
                    f'rsync'
                    f' {await GenBackendRsyncWorkspaceBuilder.get_flags(options)}'
                    f' {dst_uri.username}@{dst_uri.hostname}:{dst_uri.path / "*"}'
                    f' {await GenBackendRsyncWorkspaceBuilder.get_src_uri(options)}'
                ),
                options=options,
            )
