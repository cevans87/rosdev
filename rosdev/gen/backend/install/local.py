from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.install.local_base import GenBackendInstallLocalBase
from rosdev.gen.backend.install.mixin import GenBackendInstallMixin
from rosdev.gen.backend.rsync.home.local import GenBackendRsyncHomeLocal
from rosdev.gen.backend.rsync.home.mixin import GenBackendRsyncHomeMixin
from rosdev.util.options import Options

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendInstallLocal(GenBackendInstallMixin, GenBackendInstallLocalBase):

    @staticmethod
    @final
    @memoize
    def get_rsync_home(options: Options) -> Type[GenBackendRsyncHomeMixin]:
        rsync_home = GenBackendRsyncHomeLocal

        log.debug(f'{__class__.__name__} {rsync_home = }')

        return rsync_home
