from abc import ABC, abstractmethod
from dataclasses import dataclass
from logging import getLogger
from typing import Dict, Type

from rosdev.gen.backend.home.mixin_base import GenBackendHomeMixinBase
from rosdev.gen.docker.container import GenDockerContainer
from rosdev.gen.docker.entrypoint_sh import GenDockerEntrypointSh
from rosdev.util.atools import memoize
from rosdev.util.frozendict import frozendict, FrozenDict
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.path import Path


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendEntrypointShMixinBase(Handler, ABC):

    @staticmethod
    @abstractmethod
    def get_home_base(options: Options) -> Type[GenBackendHomeMixinBase]:
        raise NotImplementedError

    @classmethod
    @memoize
    async def get_path_by_env_key(cls, options: Options) -> FrozenDict[str, Path]:
        path_by_env_key: Dict[str, Path] = {}
        for env_key, path in (await GenDockerEntrypointSh.get_path_by_env_key(options)).items():
            path = await cls.get_home_base(options).get_path(options) / path.relative_to(Path.home())
            if 'quick_setup' in (filename := path.parts[-1]):
                path = path.parent / f'{cls.__name__}_{filename}'

            path_by_env_key[env_key] = path

        path_by_env_key: FrozenDict[str, Path] = frozendict(path_by_env_key)

        log.debug(f'{cls.__name__} {path_by_env_key = }')

        return path_by_env_key

    @classmethod
    @memoize
    async def get_path(cls, options: Options) -> Path:
        path = (
                (await cls.get_home_base(options).get_path(options)) /
                (await GenDockerEntrypointSh.get_path(options)).relative_to(Path.home())
        )

        log.debug(f'{cls.__name__} {path = }')

        return path
