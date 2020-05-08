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
        search_pattern = f'{await GenIdeaIdeBase.get_name(options)}*'
        if sys.platform == 'darwin':
            paths = (Path.home() / 'Library' / 'Preferences').glob(search_pattern)
        else:
            paths = (Path.home() / '.config' / 'JetBrains').glob(search_pattern)

        if paths:
            path = sorted(paths)[-1]
        else:
            search_pattern = f'.{await GenIdeaIdeBase.get_name(options)}*'
            paths = Path.home().glob(search_pattern)
            assert paths, (
                f'Could not find any {await GenIdeaIdeBase.get_name(options)} settings directories'
                f' in {Path.home()}'
            )
            path = sorted(paths)[-1] / 'config'

        log.debug(f'{cls.__name__} {path = }')

        return path
