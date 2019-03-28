from asyncio import create_task
from atools import memoize
from dataclasses import dataclass
from logging import getLogger
import os
import sys

from rosdev.gen.clion.environment import Environment
from rosdev.gen.clion.sshd import Sshd
from rosdev.util.handler import Handler
from rosdev.util.subprocess import get_exec_lines, shell


log = getLogger(__name__)


@memoize
@dataclass(frozen=True)
class Clion(Handler):

    @memoize
    async def _main(self) -> None:
        which_clion = create_task(get_exec_lines('which clion'))

        await Sshd(self.options)
        await Environment(self.options)

        await shell(
            f'nohup {sys.executable} {(await which_clion)[0]} '
            f'{os.getcwd()} < /dev/null > /dev/null 2>&1 &')
