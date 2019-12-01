from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.runner_packages import GenBackendAptRunnerPackages
from rosdev.gen.backend.pip.packages_mixin import GenBackendPipPackagesMixin
from rosdev.gen.backend.runner_base import GenBackendRunnerBase


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendPipRunnerPackages(GenBackendPipPackagesMixin, GenBackendRunnerBase):
    pass
