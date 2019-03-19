from asyncio import create_task
from atools import memoize
from logging import getLogger
import os
import pyperclip
import sys

from rosdev.util.handler import Handler
from rosdev.util.subprocess import get_exec_lines, get_shell_lines, shell


log = getLogger(__name__)


@memoize
class Clion(Handler):

    @memoize
    async def _main(self) -> None:
        which_clion = create_task(get_exec_lines('which clion'))

        lines = await get_shell_lines(f'env -i bash -c \'source install/setup.bash && env\'')
        for line in lines:
            if line:
                k, v = line.split('=', 1)
                os.environ[k] = v

        pyperclip.copy('\n'.join(lines).strip())
        log.info(
            'Cmake environment copied to clipboard. Paste to Settings -> Build, Execution '
            'Deployment -> CMake -> Environment.')

        # At this point, env will point us to the ROS python executable, so we need to be explicit
        # about the python executable we use to execute the clion script.
        await shell(
            f'nohup {sys.executable} {(await which_clion)[0]} '
            f'{os.getcwd()} < /dev/null > /dev/null 2>&1 &')
