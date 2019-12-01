from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.builder_packages import GenBackendAptBuilderPackages
from rosdev.gen.backend.pip.packages_mixin import GenBackendPipPackagesMixin
from rosdev.gen.backend.builder_base import GenBackendBuilderBase


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendPipBuilderPackages(GenBackendPipPackagesMixin, GenBackendBuilderBase):
    pass
