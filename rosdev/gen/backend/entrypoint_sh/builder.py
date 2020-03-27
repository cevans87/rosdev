from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.cwd.builder_base import GenBackendCwdBuilderBase
from rosdev.gen.backend.entrypoint_sh.builder_base import GenBackendEntrypointShBuilderBase
from rosdev.gen.backend.entrypoint_sh.mixin import GenBackendEntrypointShMixin
from rosdev.gen.backend.rsync.builder import GenBackendRsyncBuilder
from rosdev.gen.backend.ssh.builder import GenBackendSshBuilder
from rosdev.util.options import Options

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendEntrypointShBuilder(GenBackendEntrypointShMixin, GenBackendEntrypointShBuilderBase):

    @staticmethod
    @final
    def get_cwd_base(options: Options) -> Type[GenBackendCwdBuilderBase]:
        cwd_base = GenBackendCwdBuilderBase

        log.debug(f'{__class__.__name__} {cwd_base = }')

        return cwd_base

    @staticmethod
    @final
    def get_rsync(options: Options) -> Type[GenBackendRsyncBuilder]:
        rsync = GenBackendRsyncBuilder

        log.debug(f'{__class__.__name__} {rsync = }')

        return rsync

    @staticmethod
    @final
    def get_ssh(options: Options) -> Type[GenBackendSshBuilder]:
        ssh = GenBackendSshBuilder

        log.debug(f'{__class__.__name__} {ssh = }')
        
        return ssh
