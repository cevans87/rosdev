from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.home.local_base import GenBackendHomeLocalBase
from rosdev.gen.backend.rsync.home.mixin_base import GenBackendRsyncHomeMixinBase
from rosdev.gen.backend.rsync.local_base import GenBackendRsyncLocalBase
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRsyncHomeLocalBase(GenBackendRsyncHomeMixinBase, GenBackendRsyncLocalBase):

    @staticmethod
    @final
    @memoize
    def get_home(options: Options) -> Type[GenBackendHomeLocalBase]:
        home = GenBackendHomeLocalBase

        log.debug(f'{__class__.__name__} {home = }')

        return home
