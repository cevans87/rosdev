from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.home.local_base import GenBackendHomeLocalBase
from rosdev.gen.backend.store.mixin_base import GenBackendStoreMixinBase
from rosdev.gen.backend.local_base import GenBackendLocalBase
from rosdev.util.options import Options

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendStoreLocalBase(GenBackendStoreMixinBase, GenBackendLocalBase):

    @staticmethod
    @final
    @memoize
    def get_home_base(options: Options) -> Type[GenBackendHomeLocalBase]:
        home_base = GenBackendHomeLocalBase

        log.debug(f'{__class__.__name__} {home_base = }')

        return home_base
