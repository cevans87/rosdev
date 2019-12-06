from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.source.mixin_base import GenBackendAptSourceMixinBase
from rosdev.gen.backend.builder_base import GenBackendBuilderBase


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendAptSourceBuilderBase(GenBackendAptSourceMixinBase, GenBackendBuilderBase):
    pass
