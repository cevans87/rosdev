from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.rosdev.home.local import GenBackendRosdevHomeLocal
from rosdev.gen.backend.rosdev.home.mixin import GenBackendRosdevHomeMixin
from rosdev.gen.backend.rosdev.workspace.local_base import GenBackendRosdevWorkspaceLocalBase
from rosdev.gen.backend.rosdev.workspace.mixin import GenBackendRosdevWorkspaceMixin
from rosdev.gen.backend.rsync.workspace.local import GenBackendRsyncWorkspaceLocal
from rosdev.gen.backend.rsync.workspace.mixin import GenBackendRsyncWorkspaceMixin
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRosdevWorkspaceLocal(
    GenBackendRosdevWorkspaceMixin, GenBackendRosdevWorkspaceLocalBase
):

    @staticmethod
    @final
    @memoize
    async def get_rosdev_home(options: Options) -> Type[GenBackendRosdevHomeMixin]:
        rosdev_home = GenBackendRosdevHomeLocal
        
        log.debug(f'{__class__.__name__} {rosdev_home = }')
        
        return rosdev_home

    @staticmethod
    @final
    @memoize
    async def get_rsync_workspace(options: Options) -> Type[GenBackendRsyncWorkspaceMixin]:
        rsync_workspace = GenBackendRsyncWorkspaceLocal

        log.debug(f'{__class__.__name__} {rsync_workspace = }')

        return rsync_workspace
