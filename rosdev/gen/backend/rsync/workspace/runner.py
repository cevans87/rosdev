from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.rsync.workspace.runner_base import GenBackendRsyncWorkspaceRunnerBase
from rosdev.gen.backend.rsync.workspace.mixin import GenBackendRsyncWorkspaceMixin


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRsyncWorkspaceRunner(
    GenBackendRsyncWorkspaceMixin, GenBackendRsyncWorkspaceRunnerBase
):
    pass
