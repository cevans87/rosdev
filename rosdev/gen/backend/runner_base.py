from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.mixin_base import GenBackendMixinBase
from rosdev.gen.backend.ssh.mixin import GenBackendSshMixin
from rosdev.gen.backend.ssh.runner import GenBackendSshRunner
from rosdev.util.options import Options

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRunnerBase(GenBackendMixinBase):

    @staticmethod
    @final
    @memoize
    def get_ssh(options: Options) -> Type[GenBackendSshMixin]:
        ssh = GenBackendSshRunner

        log.debug(f'{GenBackendRunnerBase.__name__} {ssh = }')

        return ssh
