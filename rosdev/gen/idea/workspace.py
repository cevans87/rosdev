from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from uuid import UUID, uuid4

from rosdev.gen.host import GenHost
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.path import Path

log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaWorkspace(Handler):

    @classmethod
    @memoize
    async def get_path(cls, options: Options) -> Path:
        path = Path.workspace() / '.idea'

        log.debug(f'{cls.__name__} {path = }')

        return path

    @classmethod
    @memoize
    async def get_uuid(cls, options: Options) -> UUID:
        try:
            uuid = UUID((await cls.get_uuid_path(options)).read_text())
        except (FileNotFoundError, ValueError):
            uuid = uuid4()
        else:
            log.debug(f'Reloaded idea_uuid from {await cls.get_uuid_path(options)}')

        log.debug(f'{cls.__name__} {uuid}')
        
        return uuid

    @classmethod
    @memoize
    async def get_uuid_path(cls, options: Options) -> Path:
        uuid_path = await cls.get_path(options) / 'idea_uuid'

        log.debug(f'{cls.__name__} {uuid_path = }')

        return uuid_path
    
    @classmethod
    async def main(cls, options: Options) -> None:
        GenHost.write_text(
            data=f'{await cls.get_uuid(options)}',
            options=options,
            path=await cls.get_uuid_path(options),
        )
