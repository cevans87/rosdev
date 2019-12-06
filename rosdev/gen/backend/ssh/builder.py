from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.ssh.builder_base import GenBackendSshBuilderBase
from rosdev.gen.backend.ssh.mixin import GenBackendSshMixin


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendSshBuilder(GenBackendSshMixin, GenBackendSshBuilderBase):
    pass
