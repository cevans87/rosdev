from abc import ABC
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.mixin_base import GenBackendMixinBase

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendMixin(GenBackendMixinBase, ABC):
    pass
