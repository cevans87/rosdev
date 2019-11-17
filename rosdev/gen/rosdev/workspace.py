from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from pathlib import Path

from rosdev.gen.home import GenHome
from rosdev.gen.workspace import GenWorkspace
from rosdev.gen.rosdev.home import GenRosdevHome
from rosdev.util.handler import Handler
from rosdev.util.options import Options

log = getLogger(__name__)


@dataclass(frozen=True)
class GenRosdevWorkspace(Handler):

    @staticmethod
    @memoize
    async def get_path(options: Options) -> Path:
        path = (
            await GenWorkspace.get_path(options) /
            '.rosdev' /
            options.release /
            options.architecture
        )

        assert await GenHome.get_path(options) in path.parents

        log.debug(f'{GenRosdevWorkspace.__name__} {path = }')

        return path

    @classmethod
    async def main(cls, options: Options) -> None:
        if (await cls.get_path(options)).exists():
            (await cls.get_path(options)).unlink()
        (await cls.get_path(options)).parent.mkdir(parents=True, exist_ok=True)
        (await cls.get_path(options)).symlink_to(await GenRosdevHome.get_path(options))
