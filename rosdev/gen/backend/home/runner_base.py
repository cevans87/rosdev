from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.home.mixin_base import GenBackendHomeMixinBase
from rosdev.gen.backend.runner_base import GenBackendRunnerBase


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendHomeRunnerBase(GenBackendHomeMixinBase, GenBackendRunnerBase):
    pass
