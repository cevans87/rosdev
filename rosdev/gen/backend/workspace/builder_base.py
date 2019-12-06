from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.builder_base import GenBackendBuilderBase
from rosdev.gen.backend.home.builder_base import GenBackendHomeBuilderBase
from rosdev.gen.backend.home.mixin_base import GenBackendHomeMixinBase
from rosdev.gen.backend.workspace.mixin_base import GenBackendWorkspaceMixinBase
from rosdev.util.options import Options

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendWorkspaceBuilderBase(GenBackendWorkspaceMixinBase, GenBackendBuilderBase):

    @staticmethod
    @final
    @memoize
    async def get_home(options: Options) -> Type[GenBackendHomeMixinBase]:
        home = GenBackendHomeBuilderBase

        log.debug(f'{GenBackendWorkspaceBuilderBase.__name__} {home = }')

        return home
