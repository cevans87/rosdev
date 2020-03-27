from abc import ABC
from dataclasses import dataclass
from logging import getLogger
from typing import final

from rosdev.gen.backend.ssh.local import GenBackendSshLocal
from rosdev.gen.backend.rsync.mixin_base import GenBackendRsyncMixinBase
from rosdev.util.atools import memoize
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

        await cls.get_ssh(options).execute(
            command=f'mkdir -p {(await cls.get_dst_path(options))}',
            options=options,
        )
        
        dst_uri = await cls.get_dst_uri(options)

        await GenBackendSshLocal.execute(
            command=(
                f'rsync'
                f' {await cls.get_flags(options)}'
                f' {await cls.get_src_path(options) / "*"}'
                f' {dst_uri.username}@{dst_uri.hostname}:{dst_uri.path}'
            ),
            options=options,
        )
