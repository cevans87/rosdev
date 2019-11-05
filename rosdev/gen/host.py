from asyncio.subprocess import (
    create_subprocess_exec, create_subprocess_shell, PIPE, Process, STDOUT
)
from atools import memoize
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

    @classmethod
    async def _finish_process(
            cls,
            capture_output: bool,
            command: str,
            err_ok: bool,
            process: Process,
    ) -> Optional[Tuple[str]]:

        await process.wait()

        if (process.returncode != 0) and (not err_ok):
            raise cls.SubprocessException(f'Command "{command}" exited code: {process.returncode}')

        if capture_output:
            return tuple(((await process.communicate())[0]).decode().strip().splitlines())
        else:
            return None

    @classmethod
    @memoize
    async def _execute(
            cls,
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

        return await cls._finish_process(
            capture_output=capture_output,
            command=command,
            err_ok=err_ok,
            process=process,
        )

    @classmethod
    async def execute(
            cls,
            *,
            command: str,
            err_ok: bool = False,
            options: Options,
    ) -> None:
        await cls._execute(
            capture_output=False,
            command=command,
            err_ok=err_ok,
            options=options,
        )

    @classmethod
    async def execute_and_get_lines(
            cls,
            *,
            command: str,
            err_ok: bool = False,
            options: Options,
    ) -> Tuple[str]:
        return await cls._execute(
            capture_output=True,
            command=command,
            err_ok=err_ok,
            options=options,
        )

    @classmethod
    async def execute_and_get_line(
            cls,
            *,
            command: str,
            err_ok: bool = False,
            options: Options,
    ) -> str:
        lines = await cls.execute_and_get_lines(command=command, err_ok=err_ok, options=options)
        assert len(lines) == 1, f'Must contain one line: {lines = }'

        return lines[0]

    @classmethod
    @memoize
    async def _execute_shell(
            cls,
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

        return await cls._finish_process(
            process=process, command=command, err_ok=err_ok, capture_output=capture_output
        )

    @classmethod
    async def execute_shell(
            cls,
            *,
            command: str,
            err_ok: bool = False,
            options: Options,
    ) -> Tuple[str]:
        return await cls._execute_shell(
            capture_output=True,
            command=command,
            err_ok=err_ok,
            options=options,
        )

    @classmethod
    async def execute_shell_and_get_lines(
            cls,
            *,
            command: str,
            err_ok: bool = False,
            options: Options,
    ) -> Tuple[str]:
        return await cls._execute_shell(
            command=command,
            capture_output=True,
            err_ok=err_ok,
            options=options,
        )

    @classmethod
    async def execute_shell_and_get_line(
            cls,
            *,
            command: str,
            err_ok: bool = False,
            options: Options,
    ) -> str:
        lines = await cls.execute_shell_and_get_lines(
            command=command, err_ok=err_ok, options=options,
        )
        assert len(lines) == 1, f'Must contain one line: {lines = }'

        return lines[0]

    @classmethod
    def read_bytes(cls, path: Path) -> bytes:
        data = path.read_bytes()

        log.debug(f'Read from {path}, data: \n{data.decode()}')
        return data

    @classmethod
    def read_text(cls, path: Path) -> str:
        data = path.read_text()

        log.debug(f'Read from {path}, data: \n{data}')
        return data

    @classmethod
    def write_bytes(cls, *, options: Options, path: Path, data: bytes) -> None:
        assert options.stage == 'main', 'Cannot write files outside of main'

        log.debug(f'Writing to {path}, bytes:\n{data.decode()}')
        path.parent.mkdir(parents=True, exist_ok=True)
        if not options.dry_run:
            path.write_bytes(data)

    @classmethod
    def write_text(cls, *, options: Options, path: Path, data: str) -> None:
        assert options.stage == 'main', 'Cannot write files outside of main'

        log.debug(f'Writing to {path}, text: \n{data}')
        path.parent.mkdir(parents=True, exist_ok=True)
        if not options.dry_run:
            path.write_text(data)
