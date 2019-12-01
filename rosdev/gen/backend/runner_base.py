from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import Optional

from rosdev.gen.backend.base import GenBackendBase
from rosdev.gen.backend.local_base import GenBackendLocalBase
from rosdev.gen.rosdev.home import GenRosdevHome
from rosdev.util.options import Options
from rosdev.util.path import Path
from rosdev.util.uri import Uri


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRunnerBase(GenBackendBase):

    @staticmethod
    @memoize
    async def get_identity_path(options: Options) -> Optional[Path]:
        if not options.backend_runner_identity_path:
            identity_path = None
        else:
            identity_path = Path(
                options.backend_runner_identity_path
            ).expanduser().absolute()

        log.debug(f'{GenBackendRunnerBase.__name__} {identity_path = }')

        return identity_path

    @staticmethod
    @memoize
    async def get_ssh_uri(options: Options) -> Uri:
        if options.backend_runner_ssh_uri:
            uri = Uri(options.backend_runner_ssh_uri)
        elif (uri_path := await GenBackendRunnerBase.get_ssh_uri_path(options)).exists():
            uri = Uri(uri_path.read_text())
        else:
            uri = await GenBackendLocalBase.get_ssh_uri(options)

        log.debug(f'{GenBackendRunnerBase.__name__} {uri = }')

        return uri

    @staticmethod
    @memoize
    async def get_ssh_uri_path(options: Options) -> Path:
        path = await GenRosdevHome.get_path(options) / 'backend_runner_uri'

        log.debug(f'{GenBackendRunnerBase.__name__} {path = }')

        return path
