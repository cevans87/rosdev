from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.rsync.builder_base import GenBackendRsyncBuilderBase
from rosdev.gen.backend.rsync.workspace.mixin_base import GenBackendRsyncWorkspaceMixinBase
from rosdev.gen.backend.workspace.builder_base import GenBackendWorkspaceBuilderBase
from rosdev.util.atools import memoize
from rosdev.util.options import Options

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRsyncWorkspaceBuilderBase(
    GenBackendRsyncWorkspaceMixinBase, GenBackendRsyncBuilderBase
):

    @classmethod
    @final
    @memoize
    def get_workspace_base(cls, options: Options) -> Type[GenBackendWorkspaceBuilderBase]:
        workspace_base = GenBackendWorkspaceBuilderBase

        log.debug(f'{cls.__name__} {workspace_base = }')

        return workspace_base
