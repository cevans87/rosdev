from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.home.local_base import GenBackendHomeLocalBase
from rosdev.gen.backend.home.mixin_base import GenBackendHomeMixinBase
from rosdev.gen.backend.workspace.mixin_base import GenBackendWorkspaceMixinBase
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendWorkspaceLocalBase(GenBackendWorkspaceMixinBase):

    @staticmethod
    @final
    @memoize
    async def get_home(options: Options) -> Type[GenBackendHomeMixinBase]:
        home = GenBackendHomeLocalBase
        
        log.debug(f'{GenBackendWorkspaceLocalBase.__name__} {home = }')

        return home
