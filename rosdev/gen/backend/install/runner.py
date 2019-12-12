from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.install.runner_base import GenBackendInstallRunnerBase
from rosdev.gen.backend.install.mixin import GenBackendInstallMixin
from rosdev.gen.backend.rsync.home.runner import GenBackendRsyncHomeRunner
from rosdev.gen.backend.rsync.home.mixin import GenBackendRsyncHomeMixin
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendInstallRunner(GenBackendInstallMixin, GenBackendInstallRunnerBase):

    @staticmethod
    @final
    @memoize
    def get_rsync_home(options: Options) -> Type[GenBackendRsyncHomeMixin]:
        rsync_home = GenBackendRsyncHomeRunner

        log.debug(f'{GenBackendInstallRunner.__name__} {rsync_home = }')

        return rsync_home
