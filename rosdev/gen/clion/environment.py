from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from pyperclip import copy

from rosdev.util.handler import Handler
from rosdev.util.lookup import get_environ


log = getLogger(__name__)


@memoize
@dataclass(frozen=True)
class Environment(Handler):

    @memoize
    async def _main(self) -> None:
        copy('\n'.join(f'{k}={v}' for k, v in (await get_environ()).items()))
        log.info(
            'Environment copied to clipboard. '
            'Paste in "Clion | Settings | Build, Execution, Deployment > CMake | Environment"')
