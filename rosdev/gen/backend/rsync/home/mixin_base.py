from abc import ABC, abstractmethod
from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.home.mixin_base import GenBackendHomeMixinBase
from rosdev.gen.backend.rsync.mixin_base import GenBackendRsyncMixinBase
from rosdev.util.options import Options
from rosdev.util.path import Path


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRsyncHomeMixinBase(GenBackendRsyncMixinBase, ABC):

    @classmethod
    @final
    @memoize
    async def get_dst_path(cls, options: Options) -> Path:
        dst_path = await cls.get_home(options).get_path(options) / '.rosdev'

        log.debug(f'{cls.__name__} {dst_path = }')

        return dst_path

    @staticmethod
    @abstractmethod
    def get_home(options: Options) -> Type[GenBackendHomeMixinBase]:
        raise NotImplementedError

    @classmethod
    @final
    @memoize
    async def get_src_path(cls, options: Options) -> Path:
        src_path = Path.rosdev()

        log.debug(f'{cls.__name__} {src_path = }')

        return src_path
