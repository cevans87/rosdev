from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.install.mixin_base import GenBackendInstallMixinBase
from rosdev.gen.backend.home.local_base import GenBackendHomeLocalBase
from rosdev.gen.backend.home.mixin_base import GenBackendHomeMixinBase
from rosdev.gen.backend.local_base import GenBackendLocalBase
from rosdev.util.atools import memoize
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendInstallLocalBase(GenBackendInstallMixinBase, GenBackendLocalBase):

    @staticmethod
    @final
    @memoize
    def get_home_base(options: Options) -> Type[GenBackendHomeMixinBase]:
        home_base = GenBackendHomeLocalBase

        log.debug(f'{GenBackendInstallLocalBase.__name__} {home_base = }')
        
        return home_base
