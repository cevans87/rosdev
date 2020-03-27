from abc import ABC
from asyncssh import connect
from dataclasses import dataclass
from logging import getLogger
from typing import Any, List, Optional, Tuple

from rosdev.gen.backend.ssh.mixin_base import GenBackendSshMixinBase
from rosdev.util.atools import memoize
from rosdev.util.frozendict import frozendict, FrozenDict
from rosdev.util.options import Options
from rosdev.util.path import Path
from rosdev.util.uri import Uri

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendSshMixin(GenBackendSshMixinBase, ABC):

    class SshProcessException(Exception):
        pass

    # noinspection PyUnusedLocal
    @staticmethod
    @memoize
    async def _execute(
            capture_output: bool,
            command: str,
            environment: FrozenDict[str, Any],
            err_ok: bool,
            identity_path: Optional[Path],
            options: Options,
            path: Optional[Path],
            sudo,
            uri: Uri,
    ) -> Optional[Tuple[str]]:
        connect_kwargs = {'host': uri.hostname}
        if identity_path is not None:
            connect_kwargs['client_keys'] = [f'{identity_path}']
        if uri.password:
            connect_kwargs['password'] = uri.password
        if uri.port:
            connect_kwargs['port'] = uri.port
        if uri.username:
            connect_kwargs['username'] = uri.username
            
        command = (
            f'{f"cd {path} && " if path is not None else ""}'
            f'{"sudo " if sudo else ""}'
            f'bash -l -c "'
            f'{"".join(f"{k}={v} " for k, v in environment.items())} {command}"'
        )
        log.debug(f'execute {uri = } {err_ok = } {command = }')
        output_lines: List[str] = []
        async with connect(**connect_kwargs) as conn, conn.create_process(command) as process:
            async for line in process.stdout:
                line = line.rstrip()
                if capture_output:
                    log.debug(f'{uri}: {line}')
                else:
                    log.info(f'{uri}: {line}')
                output_lines.append(line)
            process = await process.wait(check=False)

        if (process.returncode != 0) and (not err_ok):
            raise GenBackendSshMixin.SshProcessException(
                f'{process.exit_status = }, {command = }, {process.stderr = }'
            )

        if capture_output:
            return tuple(output_lines)
        else:
            return None

    @classmethod
    async def execute(
            cls,
            command: str,
            options: Options,
            environment: FrozenDict[str, Any] = frozendict(),
            err_ok: bool = False,
            path: Optional[Path] = None,
            sudo: bool = False,
    ) -> None:
        await cls._execute(
            capture_output=False,
            command=command,
            environment=environment,
            err_ok=err_ok,
            identity_path=await cls.get_identity_path(options),
            options=options,
            path=path,
            sudo=sudo,
            uri=await cls.get_uri(options),
        )

    @classmethod
    async def execute_and_get_lines(
            cls,
            command: str,
            options: Options,
            environment: FrozenDict[str, Any] = frozendict(),
            err_ok: bool = False,
            path: Optional[Path] = None,
            sudo: bool = False,
    ) -> Tuple[str]:
        return await cls._execute(
            capture_output=True,
            command=command,
            identity_path=await cls.get_identity_path(options),
            environment=environment,
            err_ok=err_ok,
            options=options,
            path=path,
            sudo=sudo,
            uri=await cls.get_uri(options),
        )

    @classmethod
    async def execute_and_get_line(
            cls,
            command: str,
            options: Options,
            err_ok: bool = False,
            environment: FrozenDict[str, Any] = frozendict(),
            path: Optional[Path] = None,
            sudo: bool = False,
    ) -> str:
        lines = await cls._execute(
            capture_output=True,
            command=command,
            environment=environment,
            err_ok=err_ok,
            identity_path=await cls.get_identity_path(options),
            options=options,
            path=path,
            sudo=sudo,
            uri=await cls.get_uri(options),
        )
        assert len(lines) == 1, f'Must contain one line: {lines = }'

        return lines[0]
