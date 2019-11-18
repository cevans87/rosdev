from asyncio.subprocess import (
    create_subprocess_exec, create_subprocess_shell, PIPE, Process, STDOUT
)
from dataclasses import dataclass
from logging import getLogger
import os
from pathlib import Path
from typing import Optional, Tuple

from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenHost(Handler):

    class SubprocessException(Exception):
        pass

    @staticmethod
    async def _finish_process(
            capture_output: bool,
            command: str,
            err_ok: bool,
            process: Process,
    ) -> Optional[Tuple[str]]:

        await process.wait()

        if (process.returncode != 0) and (not err_ok):
            raise GenHost.SubprocessException(
                f'Command "{command}" exited code: {process.returncode}'
            )

        if capture_output:
            return tuple(((await process.communicate())[0]).decode().strip().splitlines())
        else:
            return None

    # noinspection PyUnusedLocal
    @staticmethod
    async def _execute(
            command: str,
            err_ok: bool,
            capture_output: bool,
            options: Options,
    ) -> Optional[Tuple[str]]:
        log.debug(f'execute err_ok={err_ok} "{command}"')
        process = await create_subprocess_exec(
            *command.split(),
            stdout=PIPE if capture_output else None,
            stderr=STDOUT if capture_output else None,
            env=os.environ,
        )

        return await GenHost._finish_process(
            capture_output=capture_output,
            command=command,
            err_ok=err_ok,
            process=process,
        )

    @staticmethod
    async def execute(
            *,
            command: str,
            err_ok: bool = False,
            options: Options,
    ) -> None:
        await GenHost._execute(
            capture_output=False,
            command=command,
            err_ok=err_ok,
            options=options,
        )

    @staticmethod
    async def execute_and_get_lines(
            *,
            command: str,
            err_ok: bool = False,
            options: Options,
    ) -> Tuple[str]:
        return await GenHost._execute(
            capture_output=True,
            command=command,
            err_ok=err_ok,
            options=options,
        )

    @staticmethod
    async def execute_and_get_line(
            *,
            command: str,
            err_ok: bool = False,
            options: Options,
    ) -> str:
        lines = await GenHost.execute_and_get_lines(command=command, err_ok=err_ok, options=options)
        assert len(lines) == 1, f'Must contain one line: {lines = }'

        return lines[0]

    # noinspection PyUnusedLocal
    @staticmethod
    async def _execute_shell(
            *,
            capture_output: bool,
            command: str,
            err_ok: bool,
            options: Options,
    ) -> Optional[Tuple[str]]:
        log.debug(f'execute_shell err_ok={err_ok} "{command}"')
        process = await create_subprocess_shell(
            command,
            stdout=PIPE if capture_output else None,
            stderr=STDOUT if capture_output else None,
            env=os.environ,
        )

        return await GenHost._finish_process(
            process=process, command=command, err_ok=err_ok, capture_output=capture_output
        )

    @staticmethod
    async def execute_shell(
            *,
            command: str,
            err_ok: bool = False,
            options: Options,
    ) -> Tuple[str]:
        return await GenHost._execute_shell(
            capture_output=True,
            command=command,
            err_ok=err_ok,
            options=options,
        )

    @staticmethod
    async def execute_shell_and_get_lines(
            *,
            command: str,
            err_ok: bool = False,
            options: Options,
    ) -> Tuple[str]:
        return await GenHost._execute_shell(
            command=command,
            capture_output=True,
            err_ok=err_ok,
            options=options,
        )

    @staticmethod
    async def execute_shell_and_get_line(
            *,
            command: str,
            err_ok: bool = False,
            options: Options,
    ) -> str:
        lines = await GenHost.execute_shell_and_get_lines(
            command=command, err_ok=err_ok, options=options,
        )
        assert len(lines) == 1, f'Must contain one line: {lines = }'

        return lines[0]

    @staticmethod
    def read_bytes(path: Path) -> bytes:
        data = path.read_bytes()

        log.debug(f'Read from {path}, data: \n{data.decode()}')
        return data

    @staticmethod
    def read_text(path: Path) -> str:
        data = path.read_text()

        log.debug(f'Read from {path}, data: \n{data}')
        return data

    @staticmethod
    def write_bytes(*, options: Options, path: Path, data: bytes) -> None:
        log.debug(f'Writing to {path}, bytes:\n{data.decode()}')
        path.parent.mkdir(parents=True, exist_ok=True)
        if not options.dry_run:
            path.write_bytes(data)

    @staticmethod
    def write_text(*, options: Options, path: Path, data: str) -> None:
        log.debug(f'Writing to {path}, text: \n{data}')
        path.parent.mkdir(parents=True, exist_ok=True)
        if not options.dry_run:
            path.write_text(data)
