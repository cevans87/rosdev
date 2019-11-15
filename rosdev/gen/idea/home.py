from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from pathlib import Path
import sys

from rosdev.gen.home import GenHome
from rosdev.gen.idea.ide_base import GenIdeaIdeBase
from rosdev.util.handler import Handler
from rosdev.util.options import Options

log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaHome(Handler):

    @classmethod
    @memoize
    async def get_path(cls, options: Options) -> Path:
        if sys.platform == 'darwin':
            search_path = await GenHome.get_path(options) / 'Library' / 'Preferences'
            search_pattern = f'{await GenIdeaIdeBase.get_name(options)}*'
        else:
            search_path = Path.home()
            search_pattern = f'.{await GenIdeaIdeBase.get_name(options)}*'
        ide_paths = sorted(search_path.glob(search_pattern))
        assert ide_paths, (
            f'Could not find any {await GenIdeaIdeBase.get_name(options)} settings directories in'
            f' {search_path}'
        )
        path = ide_paths[-1]
        if sys.platform != 'darwin':
            path /= 'config'

        log.debug(f'{cls.__name__} {path = }')

        return path
