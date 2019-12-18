from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.home.builder_base import GenBackendHomeBuilderBase
from rosdev.gen.backend.rsync.builder_base import GenBackendRsyncBuilderBase
from rosdev.gen.backend.rsync.home.mixin_base import GenBackendRsyncHomeMixinBase
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRsyncHomeBuilderBase(GenBackendRsyncHomeMixinBase, GenBackendRsyncBuilderBase):

    @staticmethod
    @final
    @memoize
    def get_home(options: Options) -> Type[GenBackendHomeBuilderBase]:
        home = GenBackendHomeBuilderBase

        log.debug(f'{__class__.__name__} {home = }')

        return home
