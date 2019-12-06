from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.ssh.runner_base import GenBackendSshRunnerBase
from rosdev.gen.backend.ssh.mixin import GenBackendSshMixin


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendSshRunner(GenBackendSshMixin, GenBackendSshRunnerBase):
    pass
