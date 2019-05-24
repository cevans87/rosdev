from atools import memoize
from dataclasses import dataclass
from logging import getLogger
import os
import sys

from rosdev.util.handler import Handler
from rosdev.util.subprocess import get_exec_lines, shell


log = getLogger(__name__)


@memoize
@dataclass(frozen=True)
class Ide(Handler):

    @memoize
    async def _main(self) -> None:
        await shell(
            f'nohup {sys.executable} {(await get_exec_lines("which clion"))[0]} '
            f'{os.getcwd()} < /dev/null > /dev/null 2>&1 &')
