from abc import ABC
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.builder_base import GenBackendBuilderBase
from rosdev.gen.backend.rsync.mixin_base import GenBackendRsyncMixinBase

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRsyncBuilderBase(GenBackendRsyncMixinBase, GenBackendBuilderBase, ABC):
    pass
