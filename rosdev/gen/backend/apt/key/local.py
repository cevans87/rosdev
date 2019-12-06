from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.key.local_base import GenBackendAptKeyLocalBase
from rosdev.gen.backend.apt.key.mixin import GenBackendAptKeyMixin


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendAptKeyLocal(GenBackendAptKeyMixin, GenBackendAptKeyLocalBase):
    pass
