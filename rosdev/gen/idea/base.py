from dataclasses import dataclass, field, replace
from logging import getLogger
from pathlib import Path
import sys
from typing import Tuple, Type

from rosdev.gen.base import GenBase
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.subprocess import get_exec_lines


log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaBase(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenBase,
    ))

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        idea_base_name = options.idea_base_name
        if idea_base_name is None:
            if 'clion' in sys.argv:
                idea_base_name = 'CLion'
            elif 'pycharm' in sys.argv:
                idea_base_name = 'PyCharm'
        assert idea_base_name in {'CLion', 'PyCharm'}, 'idea_ide_name must be CLion or PyCharm'
        
        idea_base_universal_executable_path = options.idea_base_executable_universal_path
        if idea_base_universal_executable_path is None:
            if idea_base_name == 'PyCharm':
                idea_base_universal_executable_path = Path((await get_exec_lines('which charm'))[0])
            elif idea_base_name == 'CLion':
                idea_base_universal_executable_path = Path((await get_exec_lines('which clion'))[0])
        idea_base_universal_executable_path = options.resolve_path(
            idea_base_universal_executable_path
        )

        idea_base_universal_path = options.idea_base_universal_path
        if idea_base_universal_path is None:
            if sys.platform == 'darwin':
                search_path = Path(Path.home(), 'Library', 'Preferences')
            else:
                search_path = Path.home()
            ide_paths = sorted(search_path.glob(f'.{idea_base_name}*'))
            assert (
                    ide_paths
            ), f'Could not find any {idea_base_name} settings directories in {search_path}'
            idea_base_universal_path = ide_paths[-1]
        idea_base_universal_path = options.resolve_path(idea_base_universal_path)

        idea_base_workspace_path = options.idea_base_workspace_path
        if idea_base_workspace_path is None:
            idea_base_workspace_path = Path(options.base_workspace_path, '.idea')
        idea_base_workspace_path = options.resolve_path(idea_base_workspace_path)

        return replace(
            options,
            idea_base_name=idea_base_name,
            idea_base_executable_universal_path=idea_base_universal_executable_path,
            idea_base_universal_path=idea_base_universal_path,
            idea_base_workspace_path=idea_base_workspace_path,
        )

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        # FIXME py38 debug print
        log.debug(f'idea_base_name: {options.idea_base_name}')
        log.debug(f'idea_base_universal_path: {options.idea_base_universal_path}')
        log.debug(f'idea_base_workspace_path: {options.idea_base_workspace_path}')

        assert (
                options.idea_base_name in {'CLion', 'PyCharm'}
        ), 'idea_ide_name must be CLion or PyCharm'
        assert (
                options.idea_base_universal_path is not None
        ), 'idea_base_universal_path cannot be None'
        assert (
                options.idea_base_workspace_path is not None
        ), 'idea_base_workspace_path cannot be None'
