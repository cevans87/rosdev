from dataclasses import dataclass, field, replace
from logging import getLogger
from uuid import UUID, uuid4
from typing import Tuple, Type

from rosdev.gen.host import GenHost
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaUuid(Handler):
    
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenHost,
    ))

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        idea_uuid = options.idea_uuid
        if idea_uuid is None:
            try:
                idea_uuid = UUID(options.idea_uuid_path.read_text())
                log.debug(f'Reloaded idea_uuid from {options.idea_uuid_path}')
            except (FileNotFoundError, ValueError):
                idea_uuid = uuid4()

        return replace(options, idea_uuid=idea_uuid)

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        log.debug(f'{options.idea_uuid = }')
        log.debug(f'{options.idea_uuid_path = }')

        assert options.idea_uuid is not None, f'Cannot be None: {options.idea_uuid}'

    @classmethod
    async def main(cls, options: Options) -> None:
        """Write idea_uuid to idea_uuid_workspace_path"""
        GenHost.write_text(
            data=f'{options.idea_uuid}',
            options=options,
            path=options.idea_uuid_path,
        )
