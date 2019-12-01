from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.pip.packages_mixin_base import GenBackendPipPackagesMixinBase
from rosdev.gen.backend.local_base import GenBackendLocalBase


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendPipLocalPackages(GenBackendPipPackagesMixinBase, GenBackendLocalBase):
    pass
