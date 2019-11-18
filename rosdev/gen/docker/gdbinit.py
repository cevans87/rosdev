from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from pathlib import Path
from textwrap import dedent

from rosdev.gen.home import GenHome
from rosdev.gen.host import GenHost
from rosdev.gen.rosdev.home import GenRosdevHome
from rosdev.gen.src import GenSrc
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerGdbinit(Handler):

    @staticmethod
    @memoize
    async def get_container_path(options: Options) -> Path:
        container_path = await GenHome.get_path(options) / '.gdbinit'

        log.debug(f'{GenDockerGdbinit.__name__} {container_path = }')

        return container_path

    @staticmethod
    @memoize
    async def get_home_path(options: Options) -> Path:
        home_path = await GenRosdevHome.get_path(options) / 'gdbinit'
        
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
        if (await cls.get_home_path(options)).exists():
            return

        log.info(f'Creating docker_gdbinit')
        GenHost.write_text(
            data=await cls.get_text(options),
            options=options,
            path=await cls.get_home_path(options),
        )
        log.info(f'Created docker_gdbinit')
