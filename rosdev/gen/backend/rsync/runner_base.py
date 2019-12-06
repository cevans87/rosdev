from abc import ABC
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.runner_base import GenBackendRunnerBase
from rosdev.gen.backend.rsync.mixin_base import GenBackendRsyncMixinBase

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRsyncRunnerBase(GenBackendRsyncMixinBase, GenBackendRunnerBase, ABC):
    pass
