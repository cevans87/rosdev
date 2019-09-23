from __future__ import annotations
from asyncio import gather
from asyncio.subprocess import (
    create_subprocess_exec, create_subprocess_shell, PIPE, Process, STDOUT
)
from atools import memoize
from dataclasses import dataclass, field, replace
from logging import getLogger
import os
from pathlib import Path
from typing import Optional, Tuple, Type

from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class Handler:
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=tuple())
    post_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=tuple())
    
    class SubprocessException(Exception):
        pass

    @classmethod
    async def run(cls, options: Options) -> None:
        options = replace(options, stage='resolve_options')
        options = await cls._resolve_all_options(options)

        options = replace(options, stage='validate_options')
        if options.run_validate_options:
            await cls._validate_all_options(options)
        else:
            log.warning('Skipping validate options')

        options = replace(options, stage='main')
        await cls._main_all(options)

    @classmethod
    async def _pre_resolve_options(cls, options: Options) -> Options:
        for pre_dependency in cls.pre_dependencies:
            options = await pre_dependency._resolve_all_options(options)

        return options

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        return options

    @classmethod
    async def _post_resolve_options(cls, options: Options) -> Options:
        for post_dependency in cls.post_dependencies:
            options = await post_dependency._resolve_all_options(options)

        return options

    @classmethod
    async def _resolve_all_options(cls, options: Options) -> Options:
        options = await cls._pre_resolve_options(options)
        options = await cls.resolve_options(options)
        options = await cls._post_resolve_options(options)

        return options

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        pass

    @classmethod
    @memoize
    async def _validate_all_options(cls, options: Options) -> None:
        await gather(
            *[
                pre_dependency._validate_all_options(options)
                for pre_dependency in cls.pre_dependencies
            ],
            cls.validate_options(options),
            *[
                post_dependency._validate_all_options(options)
                for post_dependency in cls.post_dependencies
            ]
        )

    @classmethod
    async def _pre_main(cls, options: Options) -> None:
        await gather(*[
            pre_dependency._main_all(options) for pre_dependency in cls.pre_dependencies
        ])

    @classmethod
    async def main(cls, options: Options) -> None:
        pass

    @classmethod
    async def _post_main(cls, options: Options) -> None:
        await gather(*[
            post_dependency._main_all(options) for post_dependency in cls.post_dependencies
        ])

    @classmethod
    @memoize
    async def _main_all(cls, options: Options) -> None:
        await cls._pre_main(options)
        # TODO debug print main's dockstring
        log.debug(f'Starting {cls.__name__}.main: {cls.main.__doc__ or "description unavailable"}')
        if options.run_main:
            await cls.main(options)
        else:
            log.warning(f'Skipping {cls.__name__}.main')
        log.debug(f'Finished {cls.__name__}.main')
        await cls._post_main(options)

    @classmethod
    async def _process_lines(
            cls, command: str, process: Process, err_ok: bool = False
    ) -> Tuple[str]:
        await process.wait()

        if (process.returncode != 0) and (not err_ok):
            raise cls.SubprocessException(f'Command "{command}" exited code: {process.returncode}')

        stdout = await process.stdout.read()

        # TODO return an async iterator that decodes lines as they come in
        return tuple(stdout.decode().strip().splitlines())

    @classmethod
    async def _finish_process(
            cls,
            process: Process,
            command: str,
            err_ok: bool,
            capture_output: bool,
    ) -> Optional[Tuple[str]]:

        await process.wait()

        if (process.returncode != 0) and (not err_ok):
            raise cls.SubprocessException(f'Command "{command}" exited code: {process.returncode}')

        if capture_output:
            return tuple((await process.stdout.read()).decode().strip().splitlines())
        else:
            return None

    @classmethod
    async def _exec(
            cls,
            command: str,
            err_ok: bool,
            capture_output: bool,
    ) -> Optional[Tuple[str]]:
        process = await create_subprocess_exec(
            *command.split(),
            stdout=PIPE if capture_output else None,
            stderr=STDOUT if capture_output else None,
            env=os.environ,
        )
        
        return await cls._finish_process(
            process=process, command=command, err_ok=err_ok, capture_output=capture_output
        )

    @classmethod
    @memoize
    async def exec_container(
            cls,
            command: str,
            err_ok: bool = False,
    ) -> None:
        log.debug(f'exec_container err_ok={err_ok} "{command}"')
        await cls._exec(
            command=f'docker exec {command}',
            err_ok=err_ok,
            capture_output=False,
        )

    @classmethod
    @memoize
    async def exec_workspace(
            cls,
            command: str,
            err_ok: bool = False,
    ) -> None:
        log.debug(f'exec_workspace err_ok={err_ok} "{command}"')
        await cls._exec(
            command=command,
            err_ok=err_ok,
            capture_output=False,
        )

    @classmethod
    @memoize
    async def get_exec_container_lines(
            cls,
            command: str,
            err_ok: bool = False,
    ) -> Tuple[str]:
        log.debug(f'get_exec_container_lines err_ok={err_ok} "{command}"')
        return await cls._exec(
            command=command,
            err_ok=err_ok,
            capture_output=True,
        )
    
    @classmethod
    @memoize
    async def get_exec_workspace_lines(
            cls,
            command: str,
            err_ok: bool = False,
    ) -> Tuple[str]:
        log.debug(f'get_exec_workspace_lines err_ok={err_ok} "{command}"')
        return await cls._exec(
            command=command,
            err_ok=err_ok,
            capture_output=True,
        )

    @classmethod
    async def _shell(
            cls,
            command: str,
            err_ok: bool,
            capture_output: bool,
    ) -> Optional[Tuple[str]]:
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
    @memoize
    async def shell_workspace(
            cls,
            command: str,
            err_ok: bool = False,
    ) -> Tuple[str]:
        log.debug(f'shell_workspace err_ok={err_ok} "{command}"')
        return await cls._shell(
            command=command,
            err_ok=err_ok,
            capture_output=True,
        )
    
    @classmethod
    @memoize
    async def get_shell_workspace_lines(
            cls,
            command: str,
            err_ok: bool = False,
    ) -> Tuple[str]:
        log.debug(f'get_shell_workspace_lines err_ok={err_ok} "{command}"')
        return await cls._shell(
            command=command,
            err_ok=err_ok,
            capture_output=True,
        )

    @classmethod
    def read_bytes(cls, path: Path) -> bytes:
        return path.read_bytes()

    @classmethod
    def read_text(cls, path: Path) -> str:
        return path.read_text()

    @classmethod
    def write_bytes(cls, path: Path, text: bytes) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(text)

    @classmethod
    def write_text(cls, path: Path, text: str) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)
