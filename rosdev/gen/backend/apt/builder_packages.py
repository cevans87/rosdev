from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.builder_key import GenBackendAptBuilderKey
from rosdev.gen.backend.apt.builder_source import GenBackendAptBuilderSource
from rosdev.gen.backend.apt.packages_mixin import GenBackendAptPackagesMixin
from rosdev.gen.backend.builder_base import GenBackendBuilderBase


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendAptBuilderPackages(GenBackendAptPackagesMixin, GenBackendBuilderBase):
    pass
