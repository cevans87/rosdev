from dataclasses import dataclass
from logging import getLogger
from textwrap import dedent

from rosdev.gen.src import GenSrc
from rosdev.util.atools import memoize
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.path import Path


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerGdbinit(Handler):

    @staticmethod
    @memoize
    async def get_container_path(options: Options) -> Path:
        container_path = Path.home() / '.gdbinit'

        log.debug(f'{GenDockerGdbinit.__name__} {container_path = }')

        return container_path

    @staticmethod
    @memoize
    async def get_path(options: Options) -> Path:
        home_path = Path.volume() / 'gdbinit'
        
        log.debug(f'{GenDockerGdbinit.__name__} {home_path = }')
        
        return home_path

    @staticmethod
    async def get_text(options: Options) -> str:
        text = dedent(f'''
                set directories {
                    ":".join([
                        f"{path.parent}"
                        for path in (await GenSrc.get_path(options)).rglob("CMakeLists.txt")
                    ])
                }
        ''').lstrip()
        
        log.debug(f'{GenDockerGdbinit.__name__} {text}')
        
        return text

    @classmethod
    async def main(cls, options: Options) -> None:
        if (await cls.get_path(options)).exists():
            return

        log.info(f'Creating docker_gdbinit')
        (await cls.get_path(options)).write_text(
            data=await cls.get_text(options),
        )
        log.info(f'Created docker_gdbinit')
