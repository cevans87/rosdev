from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.docker.image_base import GenDockerImageBase
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

    @staticmethod
    @memoize(db=Path.db(), keygen=lambda options: (options.architecture, options.release))
    async def get_id(options: Options) -> str:
        # noinspection PyShadowingBuiltins
        id = await GenDockerImageBase.get_id(options)

        log.debug(f'{__class__.__name__} {id = }')

        return id

    @staticmethod
    @memoize
    async def get_workspace_path(options: Options) -> Path:
        workspace_path = await GenRosdevWorkspace.get_path(options) / 'install'

        log.debug(f'{GenInstallBase.__name__} {workspace_path = }')

        return workspace_path
