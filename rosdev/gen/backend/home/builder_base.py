from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.home.mixin_base import GenBackendHomeMixinBase
from rosdev.gen.backend.builder_base import GenBackendBuilderBase


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendHomeBuilderBase(GenBackendHomeMixinBase, GenBackendBuilderBase):
    pass
