from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.key.mixin_base import GenBackendAptKeyMixinBase
from rosdev.gen.backend.local_base import GenBackendLocalBase


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendAptKeyLocalBase(GenBackendAptKeyMixinBase, GenBackendLocalBase):
    pass
