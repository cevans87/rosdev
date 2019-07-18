from atools import memoize
from dataclasses import dataclass
from logging import getLogger
import os

from rosdev.util.handler import Handler
from rosdev.util.subprocess import get_exec_lines, shell


log = getLogger(__name__)


@memoize
@dataclass(frozen=True)
class Ide(Handler):

    @memoize
    async def _main(self) -> None:
        log.info('Starting PyCharm IDE')
        await shell(
            f'nohup {(await get_exec_lines("which charm"))[0]} '
            f'{os.getcwd()} < /dev/null > /dev/null 2>&1 &'
        )
