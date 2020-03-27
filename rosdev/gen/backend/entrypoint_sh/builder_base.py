from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.entrypoint_sh.mixin_base import GenBackendEntrypointShMixinBase
from rosdev.gen.backend.builder_base import GenBackendBuilderBase
from rosdev.gen.backend.home.builder_base import GenBackendHomeBuilderBase
from rosdev.gen.backend.home.mixin_base import GenBackendHomeMixinBase
from rosdev.util.atools import memoize
from rosdev.util.options import Options

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendEntrypointShBuilderBase(GenBackendEntrypointShMixinBase, GenBackendBuilderBase):

    @staticmethod
    @final
    @memoize
    def get_home_base(options: Options) -> Type[GenBackendHomeMixinBase]:
        home_base = GenBackendHomeBuilderBase

        log.debug(f'{__class__.__name__} {home_base = }')

        return home_base
