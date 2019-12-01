from abc import ABC
from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.local import GenBackendLocal
from rosdev.gen.backend.rsync.workspace_mixin_base import GenBackendRsyncWorkspaceMixinBase
from rosdev.gen.workspace import GenWorkspace
from rosdev.util.options import Options

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRsyncWorkspaceMixin(GenBackendRsyncWorkspaceMixinBase, ABC):

    @classmethod
    @memoize
    async def main(cls, options: Options) -> None:
        await GenBackendLocal.execute(
            command=(
                f'rsync'
                f' {await cls.get_rsync_flags(options)}'
                f' {await GenWorkspace.get_path(options)}'
                f' {await cls.get_rsync_uri(options)}'
            ),
            options=options,
        )
