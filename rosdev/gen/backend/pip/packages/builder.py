from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.packages.builder import GenBackendAptPackagesBuilder
from rosdev.gen.backend.pip.packages.mixin import GenBackendPipPackagesMixin
from rosdev.gen.backend.pip.packages.builder_base import GenBackendPipPackagesBuilderBase


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendPipPackagesBuilder(GenBackendPipPackagesMixin, GenBackendPipPackagesBuilderBase):
    pass
