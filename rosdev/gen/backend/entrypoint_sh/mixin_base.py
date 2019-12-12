from abc import ABC, abstractmethod
from atools import memoize
from dataclasses import dataclass
from frozendict import frozendict
from logging import getLogger
from typing import Mapping, Type

from rosdev.gen.backend.home.mixin_base import GenBackendHomeMixinBase
from rosdev.gen.backend.mixin_base import GenBackendMixinBase
from rosdev.gen.docker.container import GenDockerContainer
from rosdev.gen.docker.entrypoint_sh import GenDockerEntrypointSh
from rosdev.gen.home import GenHome
from rosdev.util.options import Options
from rosdev.util.path import Path


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendEntrypointhMixinBase(GenBackendMixinBase, ABC):

    @staticmethod
    @abstractmethod
    def get_home(options: Options) -> Type[GenBackendHomeMixinBase]:
        raise NotImplementedError

    @classmethod
    @memoize
    async def get_environment(cls, options: Options) -> Mapping[str, str]:
        environment = frozendict({
            k: v.replace(
                f'{await GenHome.get_path(options)}',
                f'{await cls.get_home(options).get_path(options)}',
            ).replace(
                'quick_setup', f'{cls.__name__}_quick_setup'
            )
            for k, v in (await GenDockerEntrypointSh.get_environment(options)).items()
        })

        log.debug(f'{cls.__name__} {environment = }')

        return environment

    @classmethod
    @memoize
    async def get_path(cls, options: Options) -> Path:
        path = Path(
            f'{await GenDockerEntrypointSh.get_path(options)}'.replace(
                f'{await GenHome.get_path(options)}',
                f'{await cls.get_home(options).get_path(options)}'
            )
        )

        log.debug(f'{cls.__name__} {path = }')

        return path
