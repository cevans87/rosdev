from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.rsync.home.builder_base import GenBackendRsyncHomeBuilderBase
from rosdev.gen.backend.rsync.home.mixin import GenBackendRsyncHomeMixin


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRsyncHomeBuilder(GenBackendRsyncHomeMixin, GenBackendRsyncHomeBuilderBase):
    pass
