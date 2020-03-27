from abc import ABC, abstractmethod
from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.rsync.mixin_base import GenBackendRsyncMixinBase
from rosdev.gen.backend.workspace.mixin_base import GenBackendWorkspaceMixinBase
from rosdev.util.atools import memoize
from rosdev.util.options import Options
from rosdev.util.path import Path

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRsyncWorkspaceMixinBase(GenBackendRsyncMixinBase, ABC):

    @classmethod
    @final
    @memoize
    async def get_dst_path(cls, options: Options) -> Path:
        dst_path = await cls.get_workspace_base(options).get_path(options)

        log.debug(f'{cls.__name__} {dst_path = }')

        return dst_path

    @classmethod
    @final
    @memoize
    async def get_src_path(cls, options: Options) -> Path:
        src_path = Path.workspace()

        log.debug(f'{cls.__name__} {src_path = }')

        return src_path

    @staticmethod
    @abstractmethod
    def get_workspace_base(options: Options) -> Type[GenBackendWorkspaceMixinBase]:
        raise NotImplementedError
