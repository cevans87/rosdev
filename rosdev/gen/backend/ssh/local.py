from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.ssh.local_base import GenBackendSshLocalBase
from rosdev.gen.backend.ssh.mixin import GenBackendSshMixin
from rosdev.gen.docker.ssh import GenDockerSsh


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendSshLocal(GenBackendSshMixin, GenBackendSshLocalBase):
    pass
