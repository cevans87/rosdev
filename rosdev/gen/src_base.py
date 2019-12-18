from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.path import Path


log = getLogger(__name__)


@dataclass(frozen=True)
class GenSrcBase(Handler):

    @staticmethod
    @memoize
    async def get_container_path(options: Options) -> Path:
        container_path = Path('/opt/ros/src')

        log.debug(f'{__class__.__name__} {container_path = }')

        return container_path
    
    @staticmethod
    @memoize
    async def get_path(options: Options) -> Path:
        path = Path.store() / 'src'

        log.debug(f'{__class__.__name__} {path = }')

        return path
