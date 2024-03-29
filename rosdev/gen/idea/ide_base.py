from dataclasses import dataclass
from logging import getLogger
import sys

from rosdev.util.atools import memoize
from rosdev.util.handler import Handler
from rosdev.util.options import Options

log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaIdeBase(Handler):

    @classmethod
    @memoize
    async def get_name(cls, options: Options) -> str:
        name = options.idea_ide_name
        if not name:
            if 'clion' in sys.argv:
                name = 'CLion'
            elif 'pycharm' in sys.argv:
                name = 'PyCharm'
                
        assert name in {'CLion', 'PyCharm'}

        log.debug(f'{cls.__name__} {name = }')
        
        return name
