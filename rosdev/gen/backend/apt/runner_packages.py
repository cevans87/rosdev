from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.runner_key import GenBackendAptRunnerKey
from rosdev.gen.backend.apt.runner_source import GenBackendAptRunnerSource
from rosdev.gen.backend.apt.packages_mixin import GenBackendAptPackagesMixin
from rosdev.gen.backend.runner_base import GenBackendRunnerBase


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendAptRunnerPackages(GenBackendAptPackagesMixin, GenBackendRunnerBase):
    pass
