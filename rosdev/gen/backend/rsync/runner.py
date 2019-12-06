from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.rsync.home.runner import GenBackendRsyncHomeRunner
from rosdev.gen.backend.rsync.workspace.runner import GenBackendRsyncWorkspaceRunner
from rosdev.util.handler import Handler

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRsyncRunner(Handler):
    pass
