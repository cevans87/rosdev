from abc import ABC, abstractmethod
from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.ssh.mixin import GenBackendSshMixin
from rosdev.gen.backend.ssh.local import GenBackendSshLocal
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendMixinBase(Handler, ABC):

    @classmethod
    @final
    @memoize
    async def get_architecture(cls, options) -> str:
        architecture = {
            'x86_64': 'amd64', 'arm': 'arm32v7', 'aarch64': 'arm64v8'
        }[
            await cls.get_ssh(options).execute_and_get_line(
                command='uname -m',
                options=options,
            )
        ]
        
        log.debug(f'{cls.__name__} {architecture = }')
        
        assert architecture == options.architecture

        return architecture

    @staticmethod
    @abstractmethod
    def get_ssh(options: Options) -> Type[GenBackendSshMixin]:
        raise NotImplementedError

    @classmethod
    async def is_local(cls, options: Options) -> bool:
        return (
                await cls.get_ssh(options).get_uri(options) ==
                await GenBackendSshLocal.get_uri(options)
        )
