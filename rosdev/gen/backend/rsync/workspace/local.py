from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.rsync.workspace.local_base import GenBackendRsyncWorkspaceLocalBase
from rosdev.gen.backend.rsync.workspace.mixin import GenBackendRsyncWorkspaceMixin


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRsyncWorkspaceLocal(
    GenBackendRsyncWorkspaceMixin, GenBackendRsyncWorkspaceLocalBase
):
    pass
