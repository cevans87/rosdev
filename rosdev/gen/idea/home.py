from dataclasses import dataclass
from logging import getLogger
import sys

from rosdev.gen.idea.ide_base import GenIdeaIdeBase
from rosdev.util.atools import memoize
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.path import Path

log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaHome(Handler):

    @classmethod
    @memoize
    async def get_path(cls, options: Options) -> Path:
        if sys.platform == 'darwin':
            search_path = Path.home() / 'Library' / 'Preferences'
            search_pattern = f'{await GenIdeaIdeBase.get_name(options)}*'
        else:
            search_pattern = f'.{await GenIdeaIdeBase.get_name(options)}*'
        ide_paths = sorted(Path.home().glob(search_pattern))
        assert ide_paths, (
            f'Could not find any {await GenIdeaIdeBase.get_name(options)} settings directories in'
            f' {Path.home()}'
        )
        path = ide_paths[-1]
        if sys.platform != 'darwin':
            path /= 'config'

        log.debug(f'{cls.__name__} {path = }')

        return path
