from dataclasses import dataclass, replace
from logging import getLogger
from uuid import uuid4

from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaUuid(Handler):

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        idea_uuid = options.idea_uuid
        if idea_uuid is None:
            idea_uuid = uuid4()

        return replace(options, idea_uuid=idea_uuid)

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        # TODO py38 debug print
        log.debug(f'idea_uuid: {options.idea_uuid}')

        assert options.idea_uuid is not None, 'idea_uuid cannot be None'

        # TODO save uuid to workspace and restore on subsequent runs
