from asyncio import create_task
from atools import memoize
from dataclasses import dataclass
from logging import getLogger
import os
import sys
from typing import Optional

from rosdev.util.handler import Handler
from rosdev.util.subprocess import get_exec_lines, get_shell_lines, shell


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

        for line in await get_shell_lines(f'env -i bash -c \'source install/setup.bash && env\''):
            if line:
                k, v = line.split('=', 1)
                os.environ[k] = v

        # At this point, env will point us to the ROS python executable, so we need to be explicit
        # about the python executable we use to execute the clion script.
        await shell(
            f'nohup {sys.executable} {(await which_clion)[0]} '
            f'{os.getcwd()} < /dev/null > /dev/null 2>&1 &')
