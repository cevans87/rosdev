from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.key.mixin_base import GenBackendAptKeyMixinBase
from rosdev.gen.backend.runner_base import GenBackendRunnerBase


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendAptKeyRunnerBase(GenBackendAptKeyMixinBase, GenBackendRunnerBase):
    pass
