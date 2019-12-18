from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.home.runner_base import GenBackendHomeRunnerBase
from rosdev.gen.backend.rsync.runner_base import GenBackendRsyncRunnerBase
from rosdev.gen.backend.rsync.home.mixin_base import GenBackendRsyncHomeMixinBase
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRsyncHomeRunnerBase(GenBackendRsyncHomeMixinBase, GenBackendRsyncRunnerBase):

    @staticmethod
    @final
    @memoize
    def get_home(options: Options) -> Type[GenBackendHomeRunnerBase]:
        home = GenBackendHomeRunnerBase

        log.debug(f'{__class__.__name__} {home = }')

        return home
