from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.source_mixin_base import GenBackendAptSourceMixinBase
from rosdev.gen.backend.local_base import GenBackendLocalBase


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendAptLocalSource(GenBackendAptSourceMixinBase, GenBackendLocalBase):
    pass
