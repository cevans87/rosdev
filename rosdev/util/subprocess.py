from atools import memoize
from asyncio.subprocess import (
    create_subprocess_exec,
    create_subprocess_shell,
    PIPE,
    Process,
    STDOUT
)
from logging import getLogger
import os
from typing import Tuple


log = getLogger(__name__)


class SubprocessException(Exception):
    pass


async def _process_lines(command: str, process: Process, err_ok: bool = False) -> Tuple[str]:
    await process.wait()

    if (process.returncode != 0) and (not err_ok):
        raise SubprocessException(f'Command "{command}" exited code: {process.returncode}')

    stdout = await process.stdout.read()

    # TODO return an async iterator that decodes lines as they come in
    return tuple(stdout.decode().strip().splitlines())


@memoize
async def execute_command(command: str, err_ok: bool = False) -> int:
    log.debug(f'subprocess_exec err_ok={err_ok} "{command}"')
    process = await create_subprocess_exec(*command.split(), env=os.environ)
    await process.wait()

    if (process.returncode != 0) and (not err_ok):
        raise SubprocessException(f'Command "{command}" exited code: {process.returncode}')

    return process.returncode


@memoize
async def get_exec_lines(command: str, err_ok: bool = False) -> Tuple[str]:
    process = await create_subprocess_exec(
        *command.split(),
        stdout=PIPE,
        stderr=STDOUT,
        env=os.environ,
    )

    return await _process_lines(command=command, process=process, err_ok=err_ok)


@memoize
async def execute_shell(command: str, err_ok: bool = False) -> int:
    log.debug(f'subprocess_shell err_ok={err_ok} "{command}"')
    process = await create_subprocess_shell(command, env=os.environ)
    await process.wait()

    if (process.returncode != 0) and (not err_ok):
        raise SubprocessException(f'Command "{command}" exited code: {process.returncode}')

    return process.returncode


@memoize
async def get_shell_lines(command: str, err_ok: bool = False) -> Tuple[str]:
    log.debug(f'Getting lines from command: {command}')
    process = await create_subprocess_shell(command, stdout=PIPE, stderr=STDOUT, env=os.environ)

    return await _process_lines(command=command, process=process, err_ok=err_ok)
