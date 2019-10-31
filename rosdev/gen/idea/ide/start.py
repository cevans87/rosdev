from dataclasses import dataclass, field, replace
from logging import getLogger
from pathlib import Path
from typing import Tuple, Type

from rosdev.gen.base import GenBase
from rosdev.gen.idea.base import GenIdeaBase
from rosdev.util.handler import Handler
from rosdev.util.options import Options

log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaIdeStart(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenBase,
        GenIdeaBase,
    ))

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        idea_ide_start_path = options.idea_ide_start_path
        if idea_ide_start_path is None:
            if options.idea_ide_name == 'PyCharm':
                idea_ide_start_path = Path(await cls.execute_host_and_get_line(command='which charm'))
            elif options.idea_ide_name == 'CLion':
                idea_ide_start_path = Path(await cls.execute_host_and_get_line(command='which clion'))
        idea_ide_start_path = idea_ide_start_path.absolute()

        return replace(options, idea_ide_start_path=idea_ide_start_path)

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        assert options.idea_ide_start_path is not None, (
            f'Cannot be None: {options.idea_ide_start_path = }'
        )
        assert options.idea_ide_name is not None, f'Cannot be None: {options.idea_ide_name = }'

    @classmethod
    async def main(cls, options: Options) -> None:
        log.info(f'Starting {options.idea_ide_name} IDE')
        await cls.execute_shell_host(
            command=(
                f'nohup {options.idea_ide_start_path} {options.workspace_path} '
                f'< /dev/null > /dev/null 2>&1 &'
            )
        )
