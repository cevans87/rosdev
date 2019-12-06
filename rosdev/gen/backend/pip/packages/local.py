from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.packages.local import GenBackendAptPackagesLocal
from rosdev.gen.backend.pip.packages.mixin import GenBackendPipPackagesMixin
from rosdev.gen.backend.pip.packages.local_base import GenBackendPipPackagesLocalBase


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendPipPackagesLocal(GenBackendPipPackagesMixin, GenBackendPipPackagesLocalBase):
    pass
