from abc import ABC, abstractmethod
from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.rosdev.home.mixin import GenBackendRosdevHomeMixin
from rosdev.gen.backend.rosdev.workspace.mixin_base import GenBackendRosdevWorkspaceMixinBase
from rosdev.gen.backend.rsync.workspace.mixin import GenBackendRsyncWorkspaceMixin
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRosdevWorkspaceMixin(GenBackendRosdevWorkspaceMixinBase, ABC):

    @staticmethod
    @abstractmethod
    async def get_rosdev_home(options: Options) -> Type[GenBackendRosdevHomeMixin]:
        raise NotImplementedError
    
    @staticmethod
    @abstractmethod
    async def get_rsync_workspace(options: Options) -> Type[GenBackendRsyncWorkspaceMixin]:
        raise NotImplementedError

    @classmethod
    @final
    @memoize
    async def main(cls, options: Options) -> None:
        if await cls.is_local(options):
            return

        await (await cls.get_ssh(options)).execute(
            command=(
                f'mkdir -p {(await cls.get_path(options)).parent} &&'
                f' rm -f {await cls.get_path(options)} &&'
                f' ln -s'
                f' {await (await cls.get_rosdev_home(options)).get_path(options)}'
                f' {await cls.get_path(options)}'
            ),
            options=options,
        )
