from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.source_mixin import GenBackendAptSourceMixin
from rosdev.gen.backend.runner_base import GenBackendRunnerBase


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendAptRunnerSource(GenBackendAptSourceMixin, GenBackendRunnerBase):
    pass
