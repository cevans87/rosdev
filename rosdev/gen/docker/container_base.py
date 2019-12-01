from atools import memoize
from dataclasses import dataclass
import json
from logging import getLogger
from typing import Mapping

from rosdev.gen.home import GenHome
from rosdev.gen.host import GenHost
from rosdev.gen.workspace import GenWorkspace
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.path import Path

log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerContainerBase(Handler):

    @classmethod
    @memoize
    async def get_id(cls, options: Options) -> str:
        # noinspection PyShadowingBuiltins
        id = (await cls._get_inspect(options)).get('Image', '')

        log.debug(f'{cls.__name__} {id = }')

        return id

    @classmethod
    @memoize
    async def _get_inspect(cls, options: Options) -> Mapping:
        lines = await GenHost.execute_shell_and_get_lines(
            command=f'docker container inspect {await cls.get_name(options)} 2> /dev/null',
            options=options,
            err_ok=True,
        )
        inspect = array[0] if (array := json.loads('\n'.join(lines))) else {}

        log.debug(f'{cls.__name__} {inspect = }')

        return inspect

    @staticmethod
    @memoize
    async def get_name(options: Options) -> str:
        name = (
            f'rosdev_{options.release}_{options.architecture}_'
            f'{await GenWorkspace.get_hash(options)}'
        )

        log.debug(f'{GenDockerContainerBase.__name__} {name = }')

        return name

    @classmethod
    @memoize
    async def get_running(cls, options: Options) -> bool:
        running = (await cls._get_inspect(options)).get('State', {}).get('Running', False)

        log.debug(f'{cls.__name__} {running = }')

        return running

    @staticmethod
    @memoize
    async def get_ssh_path(options: Options) -> Path:
        ssh_path = await GenHome.get_path(options) / '.ssh'

        log.debug(f'{GenDockerContainerBase.__name__} {ssh_path = }')

        return ssh_path
