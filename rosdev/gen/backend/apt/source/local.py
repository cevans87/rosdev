from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.source.local_base import GenBackendAptSourceLocalBase
from rosdev.gen.backend.apt.source.mixin import GenBackendAptSourceMixin


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendAptSourceLocal(GenBackendAptSourceMixin, GenBackendAptSourceLocalBase):
    pass
