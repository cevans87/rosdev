from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from pathlib import Path

from rosdev.gen.rosdev.home import GenRosdevHome
from rosdev.gen.rosdev.workspace import GenRosdevWorkspace
from rosdev.util.handler import Handler
from rosdev.util.options import Options

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
    @memoize
    async def get_path(options: Options) -> Path:
        path = await GenRosdevWorkspace.get_path(options) / 'install'

        log.debug(f'{GenInstallBase.__name__} {path = }')

        return path

