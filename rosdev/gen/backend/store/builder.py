from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.store.builder_base import GenBackendStoreBuilderBase
from rosdev.gen.backend.store.mixin import GenBackendStoreMixin

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendStoreBuilder(GenBackendStoreMixin, GenBackendStoreBuilderBase):
    pass
