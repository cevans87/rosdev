from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.store.local_base import GenBackendStoreLocalBase
from rosdev.gen.backend.store.mixin import GenBackendStoreMixin

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendStoreLocal(GenBackendStoreMixin, GenBackendStoreLocalBase):
    pass
