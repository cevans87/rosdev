from asyncio import gather
from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from pathlib import Path
import sys

from rosdev.util.handler import Handler
from rosdev.util.subprocess import exec


log = getLogger(__name__)


@memoize
@dataclass(frozen=True)
class Config(Handler):

    @property
    def global_path(self) -> str:
        # FIXME find a way to determine installed clion version.
        if sys.platform == 'darwin':
            return f'{Path.home()}/Library/Preferences/.CLion2019.1/config'
        return f'{Path.home()}/.CLion2019.1/config'

    @property
    def local_path(self) -> str:
        return f'{Path.cwd()}/.idea'

    async def _main(self) -> None:
        await gather(
            exec(f'mkdir -p {self.global_path}'),
            exec(f'mkdir -p {self.local_path}'),
        )
