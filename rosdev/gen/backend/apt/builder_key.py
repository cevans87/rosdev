from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.key_mixin import GenBackendAptKeyMixin
from rosdev.gen.backend.builder_base import GenBackendBuilderBase


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendAptBuilderKey(GenBackendAptKeyMixin, GenBackendBuilderBase):
    pass
