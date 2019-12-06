from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.rosdev.workspace.mixin_base import GenBackendRosdevWorkspaceMixinBase
from rosdev.gen.backend.home.local_base import GenBackendHomeLocalBase
from rosdev.gen.backend.home.mixin_base import GenBackendHomeMixinBase
from rosdev.gen.backend.local_base import GenBackendLocalBase
from rosdev.util.options import Options

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRosdevWorkspaceLocalBase(GenBackendRosdevWorkspaceMixinBase, GenBackendLocalBase):

    @staticmethod
    @final
    @memoize
    async def get_home_base(options: Options) -> Type[GenBackendHomeMixinBase]:
        home_base = GenBackendHomeLocalBase

        log.debug(f'{GenBackendRosdevWorkspaceLocalBase.__name__} {home_base = }')

        return home_base
