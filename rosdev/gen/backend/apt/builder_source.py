from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.source_mixin import GenBackendAptSourceMixin
from rosdev.gen.backend.builder_base import GenBackendBuilderBase


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendAptBuilderSource(GenBackendAptSourceMixin, GenBackendBuilderBase):
    pass
