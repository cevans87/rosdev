from dataclasses import dataclass, field
from logging import getLogger
from typing import Tuple, Type

from rosdev.gen.base import GenBase
from rosdev.gen.idea.base import GenIdeaBase
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.subprocess import shell


log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaIde(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenBase,
        GenIdeaBase,
    ))

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        assert (
                options.idea_base_executable_universal_path is not None
        ), 'idea_ide_executable_universal_path cannot be None'
        
        assert options.idea_base_ide_name is not None, 'idea_ide_name cannot be None'

    @classmethod
    async def main(cls, options: Options) -> None:
        log.info(f'Starting {options.idea_base_ide_name} IDE')
        await shell(
            f'nohup {options.idea_base_executable_universal_path} {options.base_workspace_path} '
            f'< /dev/null > /dev/null 2>&1 &'
        )
