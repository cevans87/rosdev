from dataclasses import dataclass, field, replace
from logging import getLogger
from pathlib import Path
import sys
from typing import Tuple, Type

from rosdev.gen.idea.ide.name import GenIdeaIdeName
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaUniversal(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenIdeaIdeName,
    ))

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        idea_universal_path = options.idea_universal_path
        if idea_universal_path is None:
            if sys.platform == 'darwin':
                search_path = Path(options.home_universal_path, 'Library', 'Preferences')
            else:
                search_path = Path.home()
            ide_paths = sorted(search_path.glob(f'.{options.idea_ide_name}*'))
            assert ide_paths, (
                f'Could not find any {options.idea_ide_name} settings directories in {search_path}'
            )
            idea_universal_path = ide_paths[-1]
        idea_universal_path = options.resolve_path(idea_universal_path)

        return replace(options, idea_universal_path=idea_universal_path)

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        # FIXME py38 debug print
        log.debug(f'idea_universal_path: {options.idea_universal_path}')

        assert options.idea_universal_path is not None, 'idea_universal_path cannot be None'
