from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.key.builder import GenBackendAptKeyBuilder
from rosdev.gen.backend.apt.packages.builder import GenBackendAptPackagesBuilder
from rosdev.gen.backend.apt.source.builder import GenBackendAptSourceBuilder
from rosdev.gen.backend.builder_base import GenBackendBuilderBase
from rosdev.gen.backend.install.builder import GenBackendInstallBuilder
from rosdev.gen.backend.mixin import GenBackendMixin
from rosdev.gen.backend.pip.packages.builder import GenBackendPipPackagesBuilder
from rosdev.gen.backend.rsync.builder import GenBackendRsyncBuilder
from rosdev.gen.backend.rosdev.home.builder import GenBackendRosdevHomeBuilder
from rosdev.gen.backend.rosdev.workspace.builder import GenBackendRosdevWorkspaceBuilder


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendBuilder(GenBackendBuilderBase, GenBackendMixin):
    pass
