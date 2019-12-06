from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.source.mixin_base import GenBackendAptSourceMixinBase
from rosdev.gen.backend.runner_base import GenBackendRunnerBase


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendAptSourceRunnerBase(GenBackendAptSourceMixinBase, GenBackendRunnerBase):
    pass
