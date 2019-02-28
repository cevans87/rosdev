from asyncio import create_task
from atools import memoize
from dataclasses import dataclass
from logging import getLogger
import os
import sys
from typing import Optional

from rosdev.util.handler import Handler
from rosdev.util.subprocess import get_exec_lines, get_shell_lines


log = getLogger(__package__)


@memoize
@dataclass(frozen=True)
class Clion(Handler):
    architecture: str
    build_num: Optional[int]
    release: str

    @memoize
    async def _run(self) -> None:
        which_clion = create_task(get_exec_lines('which clion'))

        for command in [
            f'env -i bash -c \'source '
            f'.rosdev/{self.architecture}/{self.build_num or self.release}/setup.bash && env\'',

            f'env -i bash -c \'source install/setup.bash && env\'',
        ]:
            lines = await get_shell_lines(command)
            for line in lines:
                if line:
                    os.environ.setdefault(*line.split('=', 1))

        await get_shell_lines(
            f'nohup {sys.executable} {(await which_clion)[0]} '
            f'{os.getcwd()} < /dev/null > /dev/null 2>&1 &')
