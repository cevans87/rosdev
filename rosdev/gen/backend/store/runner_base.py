from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.home.runner_base import GenBackendHomeRunnerBase
from rosdev.gen.backend.store.mixin_base import GenBackendStoreMixinBase
from rosdev.gen.backend.runner_base import GenBackendRunnerBase
from rosdev.util.options import Options

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendStoreRunnerBase(GenBackendStoreMixinBase, GenBackendRunnerBase):

    @staticmethod
    @final
    @memoize
    def get_home_base(options: Options) -> Type[GenBackendHomeRunnerBase]:
        home_base = GenBackendHomeRunnerBase

        log.debug(f'{__class__.__name__} {home_base = }')

        return home_base
