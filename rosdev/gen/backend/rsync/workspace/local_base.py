from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.rsync.local_base import GenBackendRsyncLocalBase
from rosdev.gen.backend.rsync.workspace.mixin_base import GenBackendRsyncWorkspaceMixinBase
from rosdev.gen.backend.workspace.local_base import GenBackendWorkspaceLocalBase
from rosdev.gen.backend.workspace.mixin_base import GenBackendWorkspaceMixinBase
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRsyncWorkspaceLocalBase(
    GenBackendRsyncWorkspaceMixinBase, GenBackendRsyncLocalBase
):

    @classmethod
    @final
    @memoize
    def get_workspace(cls, options: Options) -> Type[GenBackendWorkspaceMixinBase]:
        home = GenBackendWorkspaceLocalBase
        
        log.debug(f'{cls.__name__} {home = }')
        
        return home
