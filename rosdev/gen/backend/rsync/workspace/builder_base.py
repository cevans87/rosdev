from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.rsync.builder_base import GenBackendRsyncBuilderBase
from rosdev.gen.backend.rsync.workspace.mixin_base import GenBackendRsyncWorkspaceMixinBase
from rosdev.gen.backend.workspace.builder_base import GenBackendWorkspaceBuilderBase
from rosdev.util.options import Options

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRsyncWorkspaceBuilderBase(
    GenBackendRsyncWorkspaceMixinBase, GenBackendRsyncBuilderBase
):

    @classmethod
    @final
    @memoize
    def get_workspace(cls, options: Options) -> Type[GenBackendWorkspaceBuilderBase]:
        home = GenBackendWorkspaceBuilderBase

        log.debug(f'{cls.__name__} {home = }')

        return home
