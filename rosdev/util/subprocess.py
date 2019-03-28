from atools import memoize
from asyncio.subprocess import (
    create_subprocess_exec,
    create_subprocess_shell,
    PIPE,
    Process
)
import os
from typing import Tuple


async def _process_lines(process: Process) -> Tuple[str]:
    await process.wait()

    stdout = await process.stdout.read()

    # FIXME return an async iterator that decodes lines as they come in
    return tuple(stdout.decode().strip().split('\n'))


@memoize
async def exec(command: str) -> int:
    process = await create_subprocess_exec(*command.split(), env=os.environ)
    await process.wait()

    return process.returncode


@memoize
async def get_exec_lines(command: str) -> Tuple[str]:
    process = await create_subprocess_exec(*command.split(), stdout=PIPE, env=os.environ)

    return await _process_lines(process)


@memoize
async def shell(command: str) -> int:
    process = await create_subprocess_shell(command, env=os.environ)
    await process.wait()

    return process.returncode


@memoize
async def get_shell_lines(command: str) -> Tuple[str]:
    process = await create_subprocess_shell(command, stdout=PIPE, env=os.environ)

    return await _process_lines(process)
