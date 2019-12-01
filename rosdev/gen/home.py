from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.path import Path

log = getLogger(__name__)


@dataclass(frozen=True)
class GenHome(Handler):

    @staticmethod
    @memoize
    async def get_path(options: Options) -> Path:
        path = Path.home()
        
        log.debug(f'{GenHome.__name__} {path = }')

        return path
