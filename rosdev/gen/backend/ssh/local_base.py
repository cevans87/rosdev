from dataclasses import dataclass
from logging import getLogger
from typing import final, Optional

from rosdev.gen.backend.ssh.mixin_base import GenBackendSshMixinBase
from rosdev.gen.docker.container import GenDockerContainer
from rosdev.util.atools import memoize
from rosdev.util.options import Options
from rosdev.util.path import Path
from rosdev.util.uri import Uri


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendSshLocalBase(GenBackendSshMixinBase):

    @staticmethod
    @final
    @memoize
    async def get_identity_path(options: Options) -> Optional[Path]:
        # FIXME generate a unique identity for the local container and use it.
        identity_path = None

        log.debug(f'{GenBackendSshLocalBase.__name__} {identity_path = }')

        return identity_path

    @staticmethod
    @final
    @memoize
    async def get_uri(options: Options) -> Uri:
        uri = await GenDockerContainer.get_uri(options)

        log.debug(f'{GenBackendSshLocalBase.__name__} {uri = }')

        return uri
