from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.home.runner_base import GenBackendHomeRunnerBase
from rosdev.gen.backend.rsync.runner_base import GenBackendRsyncRunnerBase
from rosdev.gen.backend.rsync.home.mixin_base import GenBackendRsyncHomeMixinBase
from rosdev.util.atools import memoize
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRsyncHomeRunnerBase(GenBackendRsyncHomeMixinBase, GenBackendRsyncRunnerBase):

    @staticmethod
    @final
    @memoize
    def get_home_base(options: Options) -> Type[GenBackendHomeRunnerBase]:
        home_base = GenBackendHomeRunnerBase

        log.debug(f'{__class__.__name__} {home_base = }')

        return home_base
