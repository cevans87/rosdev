from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.rosdev.home.mixin_base import GenBackendRosdevHomeMixinBase
from rosdev.gen.backend.home.builder_base import GenBackendHomeBuilderBase
from rosdev.gen.backend.home.mixin_base import GenBackendHomeMixinBase
from rosdev.gen.backend.builder_base import GenBackendBuilderBase
from rosdev.util.options import Options

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRosdevHomeBuilderBase(GenBackendRosdevHomeMixinBase, GenBackendBuilderBase):

    @staticmethod
    @final
    @memoize
    async def get_home_base(options: Options) -> Type[GenBackendHomeMixinBase]:
        home_base = GenBackendHomeBuilderBase

        log.debug(f'{GenBackendRosdevHomeBuilderBase.__name__} {home_base = }')

        return home_base
