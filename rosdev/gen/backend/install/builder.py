from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.install.builder_base import GenBackendInstallBuilderBase
from rosdev.gen.backend.install.mixin import GenBackendInstallMixin
from rosdev.gen.backend.rsync.home.builder import GenBackendRsyncHomeBuilder
from rosdev.gen.backend.rsync.home.mixin import GenBackendRsyncHomeMixin
from rosdev.util.atools import memoize
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendInstallBuilder(GenBackendInstallMixin, GenBackendInstallBuilderBase):

    @staticmethod
    @final
    @memoize
    def get_rsync_home(options: Options) -> Type[GenBackendRsyncHomeMixin]:
        rsync_home = GenBackendRsyncHomeBuilder

        log.debug(f'{GenBackendInstallBuilder.__name__} {rsync_home = }')

        return rsync_home
