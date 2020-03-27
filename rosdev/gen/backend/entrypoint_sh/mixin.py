from abc import ABC, abstractmethod
from dataclasses import dataclass
from logging import getLogger
from typing import Tuple, Type

from rosdev.gen.backend.cwd.mixin_base import GenBackendCwdMixinBase
from rosdev.gen.backend.entrypoint_sh.mixin_base import GenBackendEntrypointShMixinBase
from rosdev.gen.backend.rsync.mixin import GenBackendRsyncMixin
from rosdev.gen.backend.ssh.mixin import GenBackendSshMixin
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendEntrypointShMixin(GenBackendEntrypointShMixinBase, ABC):

    @staticmethod
    @abstractmethod
    def get_cwd_base(options: Options) -> Type[GenBackendCwdMixinBase]:
        raise NotImplementedError

    @staticmethod
    @abstractmethod
    def get_rsync(options: Options) -> Type[GenBackendRsyncMixin]:
        raise NotImplementedError

    @staticmethod
    @abstractmethod
    def get_ssh(options: Options) -> Type[GenBackendSshMixin]:
        raise NotImplementedError

    @classmethod
    async def execute(
            cls,
            command: str,
            options: Options,
            err_ok: bool = False,
            sudo: bool = False,
    ) -> None:
        await cls.get_ssh(options).execute(
            command=f'{await cls.get_path(options)} {command}',
            environment=await cls.get_path_by_env_key(options),
            err_ok=err_ok,
            options=options,
            path=await cls.get_cwd_base(options).get_path(options),
            sudo=sudo,
        )

    @classmethod
    async def execute_and_get_lines(
            cls,
            command: str,
            options: Options,
            err_ok: bool = False,
            sudo: bool = False,
    ) -> Tuple[str]:
        return await cls.get_ssh(options).execute_and_get_lines(
            command=f'{await cls.get_path(options)} {command}',
            environment=await cls.get_path_by_env_key(options),
            err_ok=err_ok,
            options=options,
            path=await cls.get_cwd_base(options).get_path(options),
            sudo=sudo,
        )

    @classmethod
    async def execute_and_get_line(
            cls,
            command: str,
            options: Options,
            err_ok: bool = False,
            sudo: bool = False,
    ) -> str:
        return await cls.get_ssh(options).execute_and_get_line(
            command=f'{await cls.get_path(options)} {command}',
            environment=await cls.get_path_by_env_key(options),
            err_ok=err_ok,
            options=options,
            # FIXME do we really want to use cwd instead of workspace?
            path=await cls.get_cwd_base(options).get_path(options),
            sudo=sudo,
        )
