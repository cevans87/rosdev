from abc import ABC, abstractmethod
from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.install.mixin_base import GenBackendInstallMixinBase
from rosdev.gen.backend.rsync.home.mixin import GenBackendRsyncHomeMixin
from rosdev.gen.install import GenInstall
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendInstallMixin(GenBackendInstallMixinBase, ABC):

    @staticmethod
    @abstractmethod
    async def get_rsync_home(options: Options) -> Type[GenBackendRsyncHomeMixin]:
        raise NotImplementedError

    @classmethod
    @final
    @memoize
    async def main(cls, options: Options) -> None:
        if await cls.is_local(options):
            return

        await (await cls.get_ssh(options)).execute(
            command=(
                f'ln -s'
                f' {await GenInstall.get_container_path(options)}'
                f' {await cls.get_path(options)}'
            ),
            err_ok=True,
            options=options,
        )
