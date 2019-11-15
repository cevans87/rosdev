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

    @classmethod
    @memoize
    async def get_path(cls, options: Options) -> Path:
        path = await GenRosdevHome.get_path(options) / 'install'

        log.debug(f'{cls.__name__} {path = }')

        return path

    @classmethod
    @memoize
    async def get_symlink_path(cls, options: Options) -> Path:
        symlink_path = await GenRosdevWorkspace.get_path(options) / 'install'

        log.debug(f'{cls.__name__} {symlink_path = }')

        return symlink_path
