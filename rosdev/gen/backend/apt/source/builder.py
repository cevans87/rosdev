from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.source.builder_base import GenBackendAptSourceBuilderBase
from rosdev.gen.backend.apt.source.mixin import GenBackendAptSourceMixin


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendAptSourceBuilder(GenBackendAptSourceMixin, GenBackendAptSourceBuilderBase):
    pass
