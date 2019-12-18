from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.runner_base import GenBackendRunnerBase
from rosdev.gen.backend.install.mixin_base import GenBackendInstallMixinBase
from rosdev.gen.backend.home.runner_base import GenBackendHomeRunnerBase
from rosdev.gen.backend.home.mixin_base import GenBackendHomeMixinBase
from rosdev.util.options import Options

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendInstallRunnerBase(GenBackendInstallMixinBase, GenBackendRunnerBase):

    @staticmethod
    @final
    @memoize
    def get_home_base(options: Options) -> Type[GenBackendHomeMixinBase]:
        home_base = GenBackendHomeRunnerBase

        log.debug(f'{GenBackendInstallRunnerBase.__name__} {home_base = }')

        return home_base
