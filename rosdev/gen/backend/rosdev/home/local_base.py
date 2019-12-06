from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.rosdev.home.mixin_base import GenBackendRosdevHomeMixinBase
from rosdev.gen.backend.home.local_base import GenBackendHomeLocalBase
from rosdev.gen.backend.home.mixin_base import GenBackendHomeMixinBase
from rosdev.gen.backend.local_base import GenBackendLocalBase
from rosdev.util.options import Options

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRosdevHomeLocalBase(GenBackendRosdevHomeMixinBase, GenBackendLocalBase):

    @staticmethod
    @final
    @memoize
    async def get_home_base(options: Options) -> Type[GenBackendHomeMixinBase]:
        home_base = GenBackendHomeLocalBase

        log.debug(f'{GenBackendRosdevHomeLocalBase.__name__} {home_base = }')

        return home_base
