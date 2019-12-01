from atools import memoize
from dataclasses import dataclass
import json
from logging import getLogger
from typing import Mapping

from rosdev.gen.host import GenHost
from rosdev.util.handler import Handler
from rosdev.util.options import Options

log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerImageBase(Handler):

    @staticmethod
    @memoize
    async def get_tag(options: Options) -> str:
        tag = f'rosdev:{options.release}_{options.architecture}'

        log.debug(f'{GenDockerImageBase.__name__} {tag = }')

        return tag

    @classmethod
    @memoize
    async def get_id(cls, options: Options) -> str:
        # noinspection PyShadowingBuiltins
        id = (await cls._get_inspect(options)).get('Id', '')

        log.debug(f'{cls.__name__} {id = }')

        return id

    @classmethod
    @memoize
    async def _get_inspect(cls, options: Options) -> Mapping:
        lines = await GenHost.execute_shell_and_get_lines(
            command=f'docker image inspect {await cls.get_tag(options)} 2> /dev/null',
            options=options,
            err_ok=True,
        )
        inspect = array[0] if (array := json.loads('\n'.join(lines))) else {}

        log.debug(f'{cls.__name__} {inspect = }')

        return inspect
