from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.rsync.home.local import GenBackendRsyncHomeLocal
from rosdev.gen.backend.rsync.workspace.local import GenBackendRsyncWorkspaceLocal
from rosdev.util.handler import Handler

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRsyncLocal(Handler):
    pass
