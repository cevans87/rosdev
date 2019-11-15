from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from pathlib import Path

from rosdev.gen.home import GenHome
from rosdev.util.handler import Handler
from rosdev.util.options import Options

log = getLogger(__name__)


@dataclass(frozen=True)
class GenRosdevHome(Handler):

    @classmethod
    @memoize
    async def get_path(cls, options: Options) -> Path:
        path = await GenHome.get_path(options) / '.rosdev' / options.release / options.architecture

        log.debug(f'{cls.__name__} {path = }')

        return path
