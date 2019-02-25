from atools import memoize
from asyncio.subprocess import (
    create_subprocess_exec,
    create_subprocess_shell,
    PIPE,
    Process
)
from typing import List


async def _process_lines(process: Process) -> List[str]:
    await process.wait()

    stdout = await process.stdout.read()

    # FIXME return an async iterator that decodes lines as they come in
    return stdout.decode().strip().split('\n')


@memoize
async def exec(command: str) -> None:
    process = await create_subprocess_exec(*command.split())
    await process.wait()


@memoize
async def get_exec_lines(command: str) -> List[str]:
    process = await create_subprocess_exec(*command.split(), stdout=PIPE)

    return await _process_lines(process)


@memoize
async def shell(command: str) -> None:
    process = await create_subprocess_shell(command)
    await process.wait()


@memoize
async def get_shell_lines(command: str) -> List[str]:
    process = await create_subprocess_shell(command, stdout=PIPE)

    return await _process_lines(process)
