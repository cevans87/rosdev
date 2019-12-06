from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.key.runner import GenBackendAptKeyRunner
from rosdev.gen.backend.apt.packages.mixin import GenBackendAptPackagesMixin
from rosdev.gen.backend.apt.source.runner import GenBackendAptSourceRunner
from rosdev.gen.backend.apt.packages.runner_base import GenBackendAptPackagesRunnerBase


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendAptPackagesRunner(GenBackendAptPackagesMixin, GenBackendAptPackagesRunnerBase):
    pass
