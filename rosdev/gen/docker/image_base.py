from atools import memoize
from dataclasses import dataclass
import json
from logging import getLogger
from typing import Mapping

from rosdev.gen.host import GenHost
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.path import Path

log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerImageBase(Handler):

    @staticmethod
    @memoize
    async def get_tag(options: Options) -> str:
        tag = f'rosdev:{options.release}_{options.architecture}'

        log.debug(f'{__class__.__name__} {tag = }')

        return tag

    @staticmethod
    @memoize
    async def get_id(options: Options) -> str:
        # noinspection PyShadowingBuiltins
        id = (await GenDockerImageBase._get_inspect(options)).get('Id', '')

        log.debug(f'{__class__.__name__} {id = }')

        return id

    @staticmethod
    @memoize(db=Path.db(), keygen=lambda options: (options.architecture, options.release))
    async def _get_inspect(options: Options) -> Mapping:
        lines = await GenHost.execute_shell_and_get_lines(
            command=(
                f'docker image inspect {await GenDockerImageBase.get_tag(options)} 2> /dev/null'
            ),
            options=options,
            err_ok=True,
        )
        inspect = array[0] if (array := json.loads('\n'.join(lines))) else {}

        log.debug(f'{__class__.__name__} {inspect = }')

        return inspect
