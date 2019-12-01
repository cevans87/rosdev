from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from textwrap import dedent

from rosdev.gen.host import GenHost
from rosdev.gen.rosdev.home import GenRosdevHome
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.path import Path


log = getLogger(__name__)


@dataclass(frozen=True)
class GenSshConfig(Handler):

    @staticmethod
    @memoize
    async def get_path(options: Options) -> Path:
        path = await GenRosdevHome.get_path(options) / 'ssh' / 'config'
        
        log.debug(f'{GenSshConfig.__name__} {path = }')

        return path

    @staticmethod
    @memoize
    async def get_text(options: Options) -> str:
        text = dedent('''
        ''').strip()

        log.debug(f'{GenSshConfig.__name__} {text = }')

        return text

    @staticmethod
    async def main(options: Options) -> None:
        if (await GenSshConfig.get_path(options)).exists():
            return
