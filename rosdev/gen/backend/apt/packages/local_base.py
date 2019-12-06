from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.packages.mixin_base import GenBackendAptPackagesMixinBase
from rosdev.gen.backend.local_base import GenBackendLocalBase


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendAptPackagesLocalBase(GenBackendAptPackagesMixinBase, GenBackendLocalBase):
    pass
