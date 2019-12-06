from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.key.local import GenBackendAptKeyLocal
from rosdev.gen.backend.apt.packages.mixin import GenBackendAptPackagesMixin
from rosdev.gen.backend.apt.source.local import GenBackendAptSourceLocal
from rosdev.gen.backend.apt.packages.local_base import GenBackendAptPackagesLocalBase


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendAptPackagesLocal(GenBackendAptPackagesMixin, GenBackendAptPackagesLocalBase):
    pass
