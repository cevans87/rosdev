from abc import ABC, abstractmethod
from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.home.mixin_base import GenBackendHomeMixinBase
from rosdev.gen.backend.mixin_base import GenBackendMixinBase
from rosdev.gen.install_base import GenInstallBase
from rosdev.util.options import Options
from rosdev.util.path import Path


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendInstallMixinBase(GenBackendMixinBase, ABC):

    @staticmethod
    @abstractmethod
    def get_home_base(options: Options) -> Type[GenBackendHomeMixinBase]:
        raise NotImplementedError

    @classmethod
    @final
    @memoize
    async def get_path(cls, options: Options) -> Path:
        path = Path(
            f'{await GenInstallBase.get_path(options)}'.replace(
                f'{Path.home()}', f'{await cls.get_home_base(options).get_path(options)}', 1
            )
        )

        log.debug(f'{cls.__name__} {path = }')

        return path
