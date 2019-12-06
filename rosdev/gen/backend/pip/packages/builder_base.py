from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.pip.packages.mixin_base import GenBackendPipPackagesMixinBase
from rosdev.gen.backend.builder_base import GenBackendBuilderBase


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendPipPackagesBuilderBase(GenBackendPipPackagesMixinBase, GenBackendBuilderBase):
    pass
