from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.key.local import GenBackendAptKeyLocal
from rosdev.gen.backend.apt.packages.local import GenBackendAptPackagesLocal
from rosdev.gen.backend.apt.source.local import GenBackendAptSourceLocal
from rosdev.gen.backend.local_base import GenBackendLocalBase
from rosdev.gen.backend.install.local import GenBackendInstallLocal
from rosdev.gen.backend.mixin import GenBackendMixin
from rosdev.gen.backend.pip.packages.local import GenBackendPipPackagesLocal
from rosdev.gen.backend.store.local import GenBackendStoreLocal
from rosdev.gen.backend.rsync.local import GenBackendRsyncLocal


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendLocal(GenBackendLocalBase, GenBackendMixin):
    pass
