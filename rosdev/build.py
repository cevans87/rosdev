from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.entrypoint_sh.builder import GenBackendEntrypointShBuilder
from rosdev.gen.backend.local import GenBackendLocal
from rosdev.gen.backend.rsync.workspace.builder import GenBackendRsyncWorkspaceBuilder
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class Build(Handler):
    """Run colcon build on build endpoint."""

    @staticmethod
    @memoize
    async def main(options: Options) -> None:
        await GenBackendEntrypointShBuilder.execute(
            command=f'colcon build{" " + options.remainder if options.remainder else ""}',
            options=options,
        )

        if not await GenBackendRsyncWorkspaceBuilder.is_local(options):
            dst_uri = await GenBackendRsyncWorkspaceBuilder.get_dst_uri(options)
            f' {dst_uri.username}@{dst_uri.hostname}{dst_uri.path}'
            await GenBackendLocal.get_ssh(options).execute(
                command=(
                    f'rsync'
                    f' {await GenBackendRsyncWorkspaceBuilder.get_flags(options)}'
                    f' {dst_uri.username}@{dst_uri.hostname}:{dst_uri.path / "*"}'
                    f' {await GenBackendRsyncWorkspaceBuilder.get_src_uri(options)}'
                ),
                options=options,
            )
