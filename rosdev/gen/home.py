from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from pathlib import Path

from rosdev.util.handler import Handler
from rosdev.util.options import Options

log = getLogger(__name__)


@dataclass(frozen=True)
class GenHome(Handler):

    @classmethod
    @memoize
    async def get_path(cls, options: Options) -> Path:
        path = Path.home()
        
        log.debug(f'{cls.__name__} {path = }')

        return path
