from abc import ABC
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.rsync.home.mixin_base import GenBackendRsyncHomeMixinBase
from rosdev.gen.backend.rsync.mixin import GenBackendRsyncMixin

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRsyncHomeMixin(GenBackendRsyncHomeMixinBase, GenBackendRsyncMixin, ABC):
    pass
