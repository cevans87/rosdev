from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.home import GenHome
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.path import Path


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerSshBase(Handler):
    
    @staticmethod
    @memoize
    async def get_path(options: Options) -> Path:
        path = await GenHome.get_path(options) / '.ssh'

        log.debug(f'{GenDockerSshBase.__name__} {path = }')

        return path