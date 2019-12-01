from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.builder_base import GenBackendBuilderBase
from rosdev.gen.backend.rsync.workspace_mixin import GenBackendRsyncWorkspaceMixin


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRsyncBuilderWorkspace(GenBackendRsyncWorkspaceMixin, GenBackendBuilderBase):
    pass
