from abc import ABC, abstractmethod
from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.home.mixin_base import GenBackendHomeMixinBase
from rosdev.gen.backend.mixin_base import GenBackendMixinBase
from rosdev.gen.home import GenHome
from rosdev.gen.rosdev.home import GenRosdevHome
from rosdev.util.options import Options
from rosdev.util.path import Path


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRosdevHomeMixinBase(GenBackendMixinBase, ABC):

    # TODO change from home_base to home
    @staticmethod
    @abstractmethod
    async def get_home_base(options: Options) -> Type[GenBackendHomeMixinBase]:
        raise NotImplementedError

    @classmethod
    @final
    @memoize
    async def get_path(cls, options: Options) -> Path:
        path = Path(
            f'{await GenRosdevHome.get_path(options)}'.replace(
                f'{await GenHome.get_path(options)}',
                f'{await (await cls.get_home_base(options)).get_path(options)}',
                1
            )
        )

        log.debug(f'{cls.__name__} {path = }')

        return path
