from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.key_mixin import GenBackendAptKeyMixin
from rosdev.gen.backend.runner_base import GenBackendRunnerBase


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendAptRunnerKey(GenBackendAptKeyMixin, GenBackendRunnerBase):
    pass
