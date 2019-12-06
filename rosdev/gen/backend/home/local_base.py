from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.home.mixin_base import GenBackendHomeMixinBase
from rosdev.gen.backend.local_base import GenBackendLocalBase


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendHomeLocalBase(GenBackendLocalBase, GenBackendHomeMixinBase):
    pass
