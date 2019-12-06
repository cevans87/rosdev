from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.key.builder_base import GenBackendAptKeyBuilderBase
from rosdev.gen.backend.apt.key.mixin import GenBackendAptKeyMixin


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendAptKeyBuilder(GenBackendAptKeyMixin, GenBackendAptKeyBuilderBase):
    pass
