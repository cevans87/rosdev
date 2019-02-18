from asyncio.subprocess import create_subprocess_shell, PIPE
from atools import memoize
from logging import getLogger
import os


log = getLogger(__name__)


@memoize
async def clion(
        architecture: str,
        release: str,
) -> None:

    for command in [
        f'env -i bash -c \'source .ros/{architecture}/{release}/setup.bash && env\'',
        f'env -i bash -c \'source install/setup.bash && env\'',
    ]:
        proc = await create_subprocess_shell(command, stdout=PIPE)
        while not proc.stdout.at_eof():
            line = await proc.stdout.readline()
            try:
                k, v = line.decode().strip().split('=', 1)
            except ValueError:
                assert proc.stdout.at_eof()
            else:
                os.environ[k] = v

    proc = await create_subprocess_shell(
        f'nohup clion {os.getcwd()} < /dev/null > /dev/null 2>&1 &')
    await proc.wait()
