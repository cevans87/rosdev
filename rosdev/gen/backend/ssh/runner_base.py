from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import final, Optional

from rosdev.gen.backend.ssh.local_base import GenBackendSshLocalBase
from rosdev.gen.backend.ssh.mixin_base import GenBackendSshMixinBase
from rosdev.util.options import Options
from rosdev.util.path import Path
from rosdev.util.uri import Uri


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendSshRunnerBase(GenBackendSshMixinBase):

    @staticmethod
    @final
    @memoize
    async def get_identity_path(options: Options) -> Optional[Path]:
        if options.backend_ssh_runner_identity_path:
            identity_path = Path(options.backend_ssh_runner_identity_path).expanduser().absolute()
        else:
            identity_path = None

        log.debug(f'{GenBackendSshRunnerBase.__name__} {identity_path = }')

        return identity_path

    @staticmethod
    @final
    @memoize
    async def get_uri(options: Options) -> Uri:
        if options.backend_ssh_runner_uri:
            uri = Uri(options.backend_ssh_runner_uri)
        else:
            uri = await GenBackendSshLocalBase.get_uri(options)

        log.debug(f'{GenBackendSshRunnerBase.__name__} {uri = }')

        return uri