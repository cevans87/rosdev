from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.rsync.home.builder import GenBackendRsyncHomeBuilder
from rosdev.gen.backend.rsync.workspace.builder import GenBackendRsyncWorkspaceBuilder
from rosdev.util.handler import Handler

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRsyncBuilder(Handler):
    pass
