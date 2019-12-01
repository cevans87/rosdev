from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.builder_key import GenBackendAptBuilderKey
from rosdev.gen.backend.apt.builder_packages import GenBackendAptBuilderPackages
from rosdev.gen.backend.apt.builder_source import GenBackendAptBuilderSource
from rosdev.gen.backend.pip.builder_packages import GenBackendPipBuilderPackages
from rosdev.gen.backend.builder_base import GenBackendBuilderBase
from rosdev.gen.backend.rsync.builder_workspace import GenBackendRsyncBuilderWorkspace


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendBuilder(GenBackendBuilderBase):
    pass
