from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.rsync.home.local_base import GenBackendRsyncHomeLocalBase
from rosdev.gen.backend.rsync.home.mixin import GenBackendRsyncHomeMixin


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRsyncHomeLocal(GenBackendRsyncHomeMixin, GenBackendRsyncHomeLocalBase):
    pass
