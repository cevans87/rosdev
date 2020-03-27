from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.cwd.runner_base import GenBackendCwdRunnerBase
from rosdev.gen.backend.entrypoint_sh.runner_base import GenBackendEntrypointShRunnerBase
from rosdev.gen.backend.entrypoint_sh.mixin import GenBackendEntrypointShMixin
from rosdev.gen.backend.ssh.runner import GenBackendSshRunner
from rosdev.gen.backend.rsync.runner import GenBackendRsyncRunner
from rosdev.util.options import Options

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendEntrypointShRunner(GenBackendEntrypointShMixin, GenBackendEntrypointShRunnerBase):

    @staticmethod
    @final
    def get_cwd_base(options: Options) -> Type[GenBackendCwdRunnerBase]:
        cwd_base = GenBackendCwdRunnerBase

        log.debug(f'{__class__.__name__} {cwd_base = }')

        return cwd_base

    @staticmethod
    @final
    def get_rsync(options: Options) -> Type[GenBackendRsyncRunner]:
        rsync = GenBackendSshRunner

        log.debug(f'{__class__.__name__} {rsync = }')

        return rsync

    @staticmethod
    @final
    def get_ssh(options: Options) -> Type[GenBackendSshRunner]:
        ssh = GenBackendSshRunner

        log.debug(f'{__class__.__name__} {ssh = }')
        
        return ssh
