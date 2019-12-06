from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.rosdev.home.runner_base import GenBackendRosdevHomeRunnerBase
from rosdev.gen.backend.rosdev.home.mixin import GenBackendRosdevHomeMixin
from rosdev.gen.backend.rsync.home.runner import GenBackendRsyncHomeRunner
from rosdev.gen.backend.rsync.home.mixin import GenBackendRsyncHomeMixin
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRosdevHomeRunner(GenBackendRosdevHomeMixin, GenBackendRosdevHomeRunnerBase):

    @staticmethod
    @final
    @memoize
    async def get_rsync_home(options: Options) -> Type[GenBackendRsyncHomeMixin]:
        rsync_home = GenBackendRsyncHomeRunner

        log.debug(f'{__class__.__name__} {rsync_home = }')

        return rsync_home
