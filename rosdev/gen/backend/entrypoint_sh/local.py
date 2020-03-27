from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.cwd.local_base import GenBackendCwdLocalBase
from rosdev.gen.backend.entrypoint_sh.local_base import GenBackendEntrypointShLocalBase
from rosdev.gen.backend.entrypoint_sh.mixin import GenBackendEntrypointShMixin
from rosdev.gen.backend.rsync.local import GenBackendRsyncLocal
from rosdev.gen.backend.ssh.local import GenBackendSshLocal
from rosdev.util.options import Options

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendEntrypointShLocal(GenBackendEntrypointShMixin, GenBackendEntrypointShLocalBase):

    @staticmethod
    @final
    def get_cwd_base(options: Options) -> Type[GenBackendCwdLocalBase]:
        cwd_base = GenBackendCwdLocalBase

        log.debug(f'{__class__.__name__} {cwd_base = }')

        return cwd_base

    @staticmethod
    @final
    def get_rsync(options: Options) -> Type[GenBackendRsyncLocal]:
        rsync = GenBackendRsyncLocal

        log.debug(f'{__class__.__name__} {rsync = }')

        return rsync

    @staticmethod
    @final
    def get_ssh(options: Options) -> Type[GenBackendSshLocal]:
        ssh = GenBackendSshLocal

        log.debug(f'{__class__.__name__} {ssh = }')
        
        return ssh
