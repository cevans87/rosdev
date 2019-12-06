from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.pip.packages.mixin_base import GenBackendPipPackagesMixinBase
from rosdev.gen.backend.runner_base import GenBackendRunnerBase


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendPipPackagesRunnerBase(GenBackendPipPackagesMixinBase, GenBackendRunnerBase):
    pass
