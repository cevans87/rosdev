from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.rosdev.home.mixin_base import GenBackendRosdevHomeMixinBase
from rosdev.gen.backend.home.runner_base import GenBackendHomeRunnerBase
from rosdev.gen.backend.home.mixin_base import GenBackendHomeMixinBase
from rosdev.gen.backend.runner_base import GenBackendRunnerBase
from rosdev.util.options import Options

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRosdevHomeRunnerBase(GenBackendRosdevHomeMixinBase, GenBackendRunnerBase):

    @staticmethod
    @final
    @memoize
    async def get_home_base(options: Options) -> Type[GenBackendHomeMixinBase]:
        home_base = GenBackendHomeRunnerBase

        log.debug(f'{GenBackendRosdevHomeRunnerBase.__name__} {home_base = }')

        return home_base
