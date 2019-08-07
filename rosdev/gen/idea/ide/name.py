from dataclasses import dataclass, field, replace
from logging import getLogger
import sys
from typing import Tuple, Type

from rosdev.gen.base import GenBase
from rosdev.util.handler import Handler
from rosdev.util.options import Options

log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaIdeName(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenBase,
    ))

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        idea_ide_name = options.idea_ide_name
        if idea_ide_name is None:
            if 'clion' in sys.argv:
                idea_ide_name = 'CLion'
            elif 'pycharm' in sys.argv:
                idea_ide_name = 'PyCharm'

        return replace(options, idea_ide_name=idea_ide_name)

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        # FIXME py38 debug print
        log.debug(f'idea_ide_name: {options.idea_ide_name}')

        assert options.idea_ide_name in {'CLion', 'PyCharm'}, (
            'idea_ide_name must be CLion or PyCharm'
        )
