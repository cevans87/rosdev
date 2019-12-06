from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.rsync.home.runner_base import GenBackendRsyncHomeRunnerBase
from rosdev.gen.backend.rsync.home.mixin import GenBackendRsyncHomeMixin


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRsyncHomeRunner(GenBackendRsyncHomeMixin, GenBackendRsyncHomeRunnerBase):
    pass
