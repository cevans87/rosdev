from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from pathlib import Path
import re

from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenWorkspace(Handler):
    
    @classmethod
    @memoize
    async def get_hash(cls, options: Options) -> str:
        relative_path = (await cls.get_path(options)).relative_to(Path.home())
        hash = re.sub(r"[^\w.]", "_", str(relative_path))
        
        log.debug(f'{cls.__name__} {hash = }')
        
        return hash

    @classmethod
    @memoize
    async def get_path(cls, options: Options) -> Path:
        path = Path.cwd()
        for parent_path in path.parents:
            if parent_path == Path.home():
                break
            if (parent_path / '.rosdev').is_dir():
                path = parent_path
                break
        
        assert path != Path.home(), 'Workspace must be in a subdirectory of home.'

        log.debug(f'{cls.__name__} {path = }')

        return path
