from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.source.runner_base import GenBackendAptSourceRunnerBase
from rosdev.gen.backend.apt.source.mixin import GenBackendAptSourceMixin


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendAptSourceRunner(GenBackendAptSourceMixin, GenBackendAptSourceRunnerBase):
    pass
