from abc import ABC
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.rsync.mixin import GenBackendRsyncMixin
from rosdev.gen.backend.rsync.workspace.mixin_base import GenBackendRsyncWorkspaceMixinBase


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRsyncWorkspaceMixin(GenBackendRsyncWorkspaceMixinBase, GenBackendRsyncMixin, ABC):
    pass
