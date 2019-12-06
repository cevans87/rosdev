from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.rsync.workspace.builder_base import GenBackendRsyncWorkspaceBuilderBase
from rosdev.gen.backend.rsync.workspace.mixin import GenBackendRsyncWorkspaceMixin


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRsyncWorkspaceBuilder(
    GenBackendRsyncWorkspaceMixin, GenBackendRsyncWorkspaceBuilderBase
):
    pass
