from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.builder_base import GenBackendBuilderBase
from rosdev.gen.backend.home.builder_base import GenBackendHomeBuilderBase
from rosdev.gen.backend.workspace.mixin_base import GenBackendWorkspaceMixinBase
from rosdev.util.options import Options

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendWorkspaceBuilderBase(GenBackendWorkspaceMixinBase, GenBackendBuilderBase):

    @staticmethod
    @final
    @memoize
    def get_home_base(options: Options) -> Type[GenBackendHomeBuilderBase]:
        home_base = GenBackendHomeBuilderBase

        log.debug(f'{__class__.__name__} {home_base = }')

        return home_base
