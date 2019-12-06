from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.key.mixin_base import GenBackendAptKeyMixinBase
from rosdev.gen.backend.builder_base import GenBackendBuilderBase


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendAptKeyBuilderBase(GenBackendAptKeyMixinBase, GenBackendBuilderBase):
    pass
