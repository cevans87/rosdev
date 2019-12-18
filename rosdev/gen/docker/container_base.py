from atools import memoize
from dataclasses import dataclass
from logging import getLogger
import re

from rosdev.gen.docker.image_base import GenDockerImageBase
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.path import Path

log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerContainerBase(Handler):

    # noinspection PyShadowingBuiltins
    @staticmethod
    @memoize
    async def get_id(options: Options) -> int:
        id = 0

        @memoize(db=True, keygen=lambda options: None, size=1)
        def get_id_inner(options: Options) -> int:
            return id + 1

        id = get_id_inner(options)
        if options.docker_container_replace:
            get_id_inner.memoize.reset_call(options)
            id = get_id_inner(options)
        
        id += await GenDockerImageBase.get_id(options)

        log.debug(f'{__class__.__name__} {id = }')

        return id

    @staticmethod
    @memoize
    async def get_name(options: Options) -> str:
        name = re.sub(
            r'[^\w.]',
            '_',
            (
                f'rosdev_{Path.workspace().relative_to(Path.home())}'
                f'_{options.release}'
                f'_{options.architecture}'
            ),
        )

        log.debug(f'{__class__.__name__} {name = }')

        return name

    @staticmethod
    @memoize
    async def get_ssh_path(options: Options) -> Path:
        ssh_path = Path.home() / '.ssh'

        log.debug(f'{__class__.__name__} {ssh_path = }')

        return ssh_path
