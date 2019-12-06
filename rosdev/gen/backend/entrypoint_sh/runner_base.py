from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.entrypoint_sh.mixin_base import GenBackendEntrypointhMixinBase
from rosdev.gen.backend.runner_base import GenBackendRunnerBase
from rosdev.gen.backend.home.runner_base import GenBackendHomeRunnerBase
from rosdev.gen.backend.home.mixin_base import GenBackendHomeMixinBase
from rosdev.util.options import Options

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendEntrypointhRunnerBase(GenBackendEntrypointhMixinBase, GenBackendRunnerBase):

    @staticmethod
    @final
    @memoize
    async def get_home(options: Options) -> Type[GenBackendHomeMixinBase]:
        home = GenBackendHomeRunnerBase

        log.debug(f'{GenBackendEntrypointhRunnerBase.__name__} {home = }')

        return home
