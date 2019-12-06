from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.packages.runner import GenBackendAptPackagesRunner
from rosdev.gen.backend.pip.packages.mixin import GenBackendPipPackagesMixin
from rosdev.gen.backend.pip.packages.runner_base import GenBackendPipPackagesRunnerBase


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendPipPackagesRunner(GenBackendPipPackagesMixin, GenBackendPipPackagesRunnerBase):
    pass
