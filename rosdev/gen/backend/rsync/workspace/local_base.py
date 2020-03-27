from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.rsync.local_base import GenBackendRsyncLocalBase
from rosdev.gen.backend.rsync.workspace.mixin_base import GenBackendRsyncWorkspaceMixinBase
from rosdev.gen.backend.workspace.local_base import GenBackendWorkspaceLocalBase
from rosdev.util.atools import memoize
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRsyncWorkspaceLocalBase(
    GenBackendRsyncWorkspaceMixinBase, GenBackendRsyncLocalBase
):

    @classmethod
    @final
    @memoize
    def get_workspace_base(cls, options: Options) -> Type[GenBackendWorkspaceLocalBase]:
        workspace_base = GenBackendWorkspaceLocalBase
        
        log.debug(f'{cls.__name__} {workspace_base = }')
        
        return workspace_base
