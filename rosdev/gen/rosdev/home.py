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
class GenRosdevHome(Handler):

    @staticmethod
    @memoize
    async def get_path(options: Options) -> Path:
        path = (
            await GenHome.get_path(options) /
            '.rosdev' /
            (await GenWorkspace.get_path(options)).relative_to(await GenHome.get_path(options)) /
            options.release /
            options.architecture
        )

        log.debug(f'{GenRosdevHome.__name__} {path = }')

        return path

    @classmethod
    async def main(cls, options: Options) -> None:
        (await cls.get_path(options)).mkdir(parents=True, exist_ok=True)
