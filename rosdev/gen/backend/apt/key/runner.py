from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.key.runner_base import GenBackendAptKeyRunnerBase
from rosdev.gen.backend.apt.key.mixin import GenBackendAptKeyMixin


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendAptKeyRunner(GenBackendAptKeyMixin, GenBackendAptKeyRunnerBase):
    pass
