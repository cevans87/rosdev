from abc import ABC
from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.base import GenBackendBase
from rosdev.gen.backend.local import GenBackendLocal
from rosdev.gen.rosdev.home import GenRosdevHome
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRsyncUserMixinBase(GenBackendBase, ABC):
    
    @classmethod
    @memoize
    async def get_flags(cls, options: Options) -> str:
        if await cls.get_identity_path(options) is not None:
            flags = f'-e "ssh -i {cls.get_identity_path(options)}"'
        else:
            flags = ''
            
        log.debug(f'{cls.__name__} {flags = }')
        
        return flags

    @classmethod
    @memoize
    async def main(cls, options: Options) -> None:
        await GenBackendLocal.execute(
            command=(
                f'rsync'
                f'{" " + await cls.get_flags(options) if cls.get_flags(options) else ""}'
                f' {await GenRosdevHome.get_path(options)}'
                f' {await cls.get_ssh_uri(options)}:{await GenRosdevHome.get_path(options)}'
            ),
            options=options,
        )
