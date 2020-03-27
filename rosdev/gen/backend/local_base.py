from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.mixin_base import GenBackendMixinBase
from rosdev.gen.backend.ssh.local import GenBackendSshLocal
from rosdev.gen.backend.ssh.mixin import GenBackendSshMixin
from rosdev.util.atools import memoize
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendLocalBase(GenBackendMixinBase):

    @staticmethod
    @final
    @memoize
    def get_ssh(options: Options) -> Type[GenBackendSshMixin]:
        ssh = GenBackendSshLocal
        
        log.debug(f'{GenBackendLocalBase.__name__} {ssh = }')

        return ssh
