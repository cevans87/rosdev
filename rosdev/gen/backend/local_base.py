from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import Optional

from rosdev.gen.backend.base import GenBackendBase
from rosdev.gen.docker.container import GenDockerContainer
from rosdev.gen.docker.ssh import GenDockerSsh
from rosdev.gen.rosdev.home import GenRosdevHome
from rosdev.util.options import Options
from rosdev.util.path import Path
from rosdev.util.uri import Uri


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendLocalBase(GenBackendBase):

    @staticmethod
    @memoize
    async def get_identity_path(options: Options) -> Optional[Path]:
        identity_path = None

        log.debug(f'{GenBackendLocalBase.__name__} {identity_path = }')

        return identity_path

    @staticmethod
    @memoize
    async def get_ssh_uri(options: Options) -> Uri:
        uri = await GenDockerContainer.get_uri(options)

        log.debug(f'{GenBackendLocalBase.__name__} {uri = }')

        return uri

    @staticmethod
    @memoize
    async def get_ssh_uri_path(options: Options) -> Path:
        path = await GenRosdevHome.get_path(options) / 'backend_local_endpoint_uri'

        log.debug(f'{GenBackendLocalBase.__name__} {path = }')

        return path
