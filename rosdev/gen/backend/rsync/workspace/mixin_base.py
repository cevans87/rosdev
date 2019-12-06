from abc import ABC, abstractmethod
from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.rsync.mixin_base import GenBackendRsyncMixinBase
from rosdev.gen.backend.workspace.mixin_base import GenBackendWorkspaceMixinBase
from rosdev.gen.workspace import GenWorkspace
from rosdev.util.options import Options
from rosdev.util.path import Path

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRsyncWorkspaceMixinBase(GenBackendRsyncMixinBase, ABC):

    @classmethod
    @final
    @memoize
    async def get_dst_path(cls, options: Options) -> Path:
        dst_path = await (await cls.get_workspace(options)).get_path(options)

        log.debug(f'{cls.__name__} {dst_path = }')

        return dst_path

    @classmethod
    @final
    @memoize
    async def get_src_path(cls, options: Options) -> Path:
        src_path = await GenWorkspace.get_path(options)

        log.debug(f'{cls.__name__} {src_path = }')

        return src_path

    @staticmethod
    @abstractmethod
    async def get_workspace(options: Options) -> Type[GenBackendWorkspaceMixinBase]:
        ...
