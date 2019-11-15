from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from pathlib import Path

from rosdev.gen.home import GenHome
from rosdev.gen.workspace import GenWorkspace
from rosdev.util.handler import Handler
from rosdev.util.options import Options

log = getLogger(__name__)


@dataclass(frozen=True)
class GenRosdevWorkspace(Handler):

    @classmethod
    @memoize
    async def get_path(cls, options: Options) -> Path:
        path = (
            await GenWorkspace.get_path(options) /
            '.rosdev' /
            options.release /
            options.architecture
        )

        assert await GenHome.get_path(options) in path.parents

        log.debug(f'{cls.__name__} {path = }')

        return path
