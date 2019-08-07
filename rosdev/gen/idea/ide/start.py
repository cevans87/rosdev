from dataclasses import dataclass, field, replace
from logging import getLogger
from pathlib import Path
from typing import Tuple, Type

from rosdev.gen.base import GenBase
from rosdev.gen.idea.base import GenIdeaBase
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.subprocess import get_exec_lines, shell

log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaIdeStart(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenBase,
        GenIdeaBase,
    ))

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        idea_ide_start_universal_path = options.idea_ide_start_universal_path
        if idea_ide_start_universal_path is None:
            if options.idea_ide_name == 'PyCharm':
                idea_ide_start_universal_path = Path((await get_exec_lines('which charm'))[0])
            elif options.idea_ide_name == 'CLion':
                idea_ide_start_universal_path = Path((await get_exec_lines('which clion'))[0])
        idea_ide_start_universal_path = options.resolve_path(
            idea_ide_start_universal_path
        )

        return replace(options, idea_ide_start_universal_path=idea_ide_start_universal_path)

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        assert options.idea_ide_start_universal_path is not None, (
            'idea_ide_executable_universal_path cannot be None'
        )

        assert options.idea_ide_name is not None, 'idea_ide_name cannot be None'

    @classmethod
    async def main(cls, options: Options) -> None:
        log.info(f'Starting {options.idea_ide_name} IDE')
        await shell(
            f'nohup {options.idea_ide_start_universal_path} {options.workspace_path} '
            f'< /dev/null > /dev/null 2>&1 &'
        )
