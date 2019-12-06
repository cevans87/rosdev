from abc import ABC
from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import final

from rosdev.gen.backend.ssh.local import GenBackendSshLocal
from rosdev.gen.backend.rsync.mixin_base import GenBackendRsyncMixinBase
from rosdev.util.options import Options

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRsyncMixin(GenBackendRsyncMixinBase, ABC):

    @classmethod
    @final
    @memoize
    async def main(cls, options: Options) -> None:
        if await cls.is_local(options):
            return

        await GenBackendSshLocal.execute(
            command=(
                f'rsync'
                f' {await cls.get_flags(options)}'
                f' {await cls.get_src_path(options)}/*'
                f' {await cls.get_dst_uri(options)}'
            ),
            options=options,
        )
