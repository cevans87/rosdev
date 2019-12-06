from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.entrypoint_sh.mixin_base import GenBackendEntrypointhMixinBase
from rosdev.gen.backend.local_base import GenBackendLocalBase
from rosdev.gen.backend.home.local_base import GenBackendHomeLocalBase
from rosdev.gen.backend.home.mixin_base import GenBackendHomeMixinBase
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendEntrypointhLocalBase(GenBackendEntrypointhMixinBase, GenBackendLocalBase):

    @staticmethod
    @final
    @memoize
    async def get_home(options: Options) -> Type[GenBackendHomeMixinBase]:
        home = GenBackendHomeLocalBase
        
        log.debug(f'{GenBackendEntrypointhLocalBase.__name__} {home = }')
        
        return home
