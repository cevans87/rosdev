from abc import ABC
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.local_base import GenBackendLocalBase
from rosdev.gen.backend.rsync.mixin_base import GenBackendRsyncMixinBase

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRsyncLocalBase(GenBackendRsyncMixinBase, GenBackendLocalBase, ABC):
    pass
