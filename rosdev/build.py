from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.builder import GenBackendBuilder
from rosdev.gen.backend.local import GenBackendLocal
from rosdev.gen.backend.rsync.builder_workspace import GenBackendRsyncBuilderWorkspace
from rosdev.gen.workspace import GenWorkspace

from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class Build(Handler):
    """Run colcon build on build endpoint."""

    @staticmethod
    @memoize
    async def main(options: Options) -> None:
        lines = await GenBackendBuilder.execute_and_get_lines(
            command='env',
            options=options,
        )
        log.info(lines)
        await GenBackendBuilder.execute(
            command=(
                f'colcon build'
                f'{" " + " ".join(options.remainder) if options.remainder else ""}'
            ),
            options=options,
        )
        await GenBackendLocal.execute(
            command=(
                f'rsync'
                f' {await GenBackendRsyncBuilderWorkspace.get_rsync_flags(options)}'
                f' {await GenWorkspace.get_path(options)}'
                f' {await GenBackendRsyncBuilderWorkspace.get_rsync_uri(options)}'
            ),
            options=options,
        )
