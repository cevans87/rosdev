from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.host import GenHost
from rosdev.gen.rosdev.home import GenRosdevHome
from rosdev.gen.rosdev.workspace import GenRosdevWorkspace
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.path import Path

log = getLogger(__name__)


@dataclass(frozen=True)
class GenInstallBase(Handler):

    @staticmethod
    @memoize
    async def get_home_path(options: Options) -> Path:
        home_path = await GenRosdevHome.get_path(options) / 'install'

        log.debug(f'{GenInstallBase.__name__} {home_path = }')

        return home_path

    @classmethod
    @memoize
    async def get_id(cls, options: Options) -> str:
        id_path = await cls.get_id_path(options)
        # noinspection PyShadowingBuiltins
        id = GenHost.read_text(id_path) if id_path.exists() else ''

        log.debug(f'{cls.__name__} {id = }')

        return id

    @staticmethod
    @memoize
    async def get_id_path(options: Options) -> Path:
        id_path = await GenRosdevHome.get_path(options) / 'install_id'

        log.debug(f'{GenInstallBase.__name__} {id_path = }')

        return id_path

    @staticmethod
    @memoize
    async def get_path(options: Options) -> Path:
        path = await GenRosdevWorkspace.get_path(options) / 'install'

        log.debug(f'{GenInstallBase.__name__} {path = }')

        return path
