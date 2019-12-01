from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.runner_key import GenBackendAptRunnerKey
from rosdev.gen.backend.apt.runner_packages import GenBackendAptRunnerPackages
from rosdev.gen.backend.apt.runner_source import GenBackendAptRunnerSource
from rosdev.gen.backend.pip.runner_packages import GenBackendPipRunnerPackages
from rosdev.gen.backend.runner_base import GenBackendRunnerBase


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRunner(GenBackendRunnerBase):
    pass
