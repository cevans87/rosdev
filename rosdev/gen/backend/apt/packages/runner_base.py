from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.packages.mixin_base import GenBackendAptPackagesMixinBase
from rosdev.gen.backend.runner_base import GenBackendRunnerBase


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendAptPackagesRunnerBase(GenBackendAptPackagesMixinBase, GenBackendRunnerBase):
    pass
