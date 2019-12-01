from abc import ABC
from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.base import GenBackendBase
from rosdev.gen.workspace import GenWorkspace
from rosdev.util.options import Options
from rosdev.util.uri import Uri

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRsyncWorkspaceMixinBase(GenBackendBase, ABC):

    @classmethod
    @memoize
    async def get_rsync_flags(cls, options: Options) -> str:
        rsync_flags_parts = ['--recursive']
        if await cls.get_identity_path(options) is not None:
            rsync_flags_parts.append(f'-e "ssh -i {await cls.get_identity_path(options)}"')
            
        rsync_flags = ' '.join(rsync_flags_parts)

        log.debug(f'{cls.__name__} {rsync_flags = }')

        return rsync_flags

    @classmethod
    @memoize
    async def get_rsync_uri(cls, options: Options) -> Uri:
        ssh_uri = await cls.get_ssh_uri(options)
        rsync_uri = Uri(
            f'{ssh_uri.username}@{ssh_uri.hostname}:{(await GenWorkspace.get_path(options)).parent}'
        )
        
        log.debug(f'{cls.__name__} {rsync_uri = }')
        
        return rsync_uri
