from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.key.builder import GenBackendAptKeyBuilder
from rosdev.gen.backend.apt.packages.mixin import GenBackendAptPackagesMixin
from rosdev.gen.backend.apt.source.builder import GenBackendAptSourceBuilder
from rosdev.gen.backend.apt.packages.builder_base import GenBackendAptPackagesBuilderBase


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendAptPackagesBuilder(GenBackendAptPackagesMixin, GenBackendAptPackagesBuilderBase):
    pass
