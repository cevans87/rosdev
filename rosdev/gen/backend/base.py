from abc import ABC, abstractmethod
from asyncssh import connect
from atools import memoize
from dataclasses import dataclass
from frozendict import frozendict
from logging import getLogger
from typing import Mapping, Optional, Tuple

from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.path import Path
from rosdev.util.uri import Uri


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendBase(Handler, ABC):

    class SshProcessException(Exception):
        pass

    # noinspection PyUnusedLocal
    @staticmethod
    @memoize
    async def _execute(
            capture_output: bool,
            command: str,
            err_ok: bool,
            identity_path: Optional[Path],
            options: Options,
            uri: Uri,
    ) -> Optional[Tuple[str]]:
        connect_kwargs = {'host': uri.hostname}
        if identity_path is not None:
            connect_kwargs['client_keys'] = [f'{identity_path}']
        if uri.port:
            connect_kwargs['port'] = uri.port
        if uri.username:
            connect_kwargs['username'] = uri.username
        async with connect(**connect_kwargs) as conn, conn.create_process(command) as process:
            if not capture_output:
                async for line in process.stdout:
                    log.info(f'{uri}: {line}')
            process = await process.wait(check=False)

        if (process.returncode != 0) and (not err_ok):
            raise GenBackendBase.SshProcessException(
                f'Command "{command}" exited {process.exit_status = }, {process.stderr = }'
            )

        if capture_output:
            return tuple(process.stdout.strip().splitlines())
        else:
            return None

    @classmethod
    async def execute(
            cls,
            command: str,
            options: Options,
            err_ok=False,
    ) -> None:
        await cls._execute(
            capture_output=False,
            command=command,
            identity_path=await cls.get_identity_path(options),
            options=options,
            err_ok=err_ok,
            uri=await cls.get_ssh_uri(options),
        )

    @classmethod
    async def execute_and_get_lines(
            cls,
            command: str,
            options: Options,
            err_ok=False,
    ) -> Tuple[str]:
        return await cls._execute(
            capture_output=True,
            command=command,
            identity_path=await cls.get_identity_path(options),
            options=options,
            err_ok=err_ok,
            uri=await cls.get_ssh_uri(options),
        )

    @classmethod
    async def execute_and_get_line(
            cls,
            command: str,
            options: Options,
            err_ok=False,
    ) -> str:
        lines = await cls._execute(
            capture_output=True,
            command=command,
            identity_path=await cls.get_identity_path(options),
            options=options,
            err_ok=err_ok,
            uri=await cls.get_ssh_uri(options),
        )
        assert len(lines) == 1, f'Must contain one line: {lines = }'

        return lines[0]

    @classmethod
    @memoize
    async def get_architecture(cls, options: Options) -> str:
        architecture = {'x86_64': 'amd64', 'arm': 'arm32v7', 'aarch64': 'arm64v8'}[
            await cls.execute_and_get_line(command='uname -m', options=options)
        ]

        log.debug(f'{cls.__name__} {architecture = }')

        return architecture

    @staticmethod
    @abstractmethod
    async def get_identity_path(options: Options) -> Optional[Path]:
        ...

    @classmethod
    @memoize
    async def get_os_release(cls, options: Options) -> Mapping[str, str]:
        os_release = frozendict({
            k: v
            for line in await cls.execute_and_get_lines(
                command='cat /etc/os-release',
                options=options,
            )
            for (k, v) in [line.split('=', 1)]
        })

        log.debug(f'{cls.__name__} {os_release = }')

        return os_release

    @classmethod
    @memoize
    async def get_os_release_version_id(cls, options: Options) -> str:
        os_release_version_id = (await cls.get_os_release(options)).get('VERSION_ID', '')

        log.debug(f'{cls.__name__} {os_release_version_id = }')

        return os_release_version_id

    @staticmethod
    @abstractmethod
    async def get_ssh_uri(options: Options) -> Uri:
        ...

    @staticmethod
    @abstractmethod
    async def get_ssh_uri_path(options: Options) -> Path:
        ...
