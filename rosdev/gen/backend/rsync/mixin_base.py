from abc import ABC, abstractmethod
from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import final

from rosdev.gen.backend.mixin_base import GenBackendMixinBase
from rosdev.util.options import Options
from rosdev.util.path import Path
from rosdev.util.uri import Uri


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRsyncMixinBase(GenBackendMixinBase, ABC):

    @staticmethod
    @abstractmethod
    async def get_dst_path(options: Options) -> Path:
        raise NotImplementedError

    @classmethod
    @final
    @memoize
    async def get_dst_uri(cls, options: Options) -> Uri:
        ssh_uri = await (await cls.get_ssh(options)).get_uri(options)
        dst_uri = Uri(
            f'rsync://{ssh_uri.username}@{ssh_uri.hostname}{(await cls.get_dst_path(options))}'
        )

        log.debug(f'{cls.__name__} {dst_uri = }')

        return dst_uri

    @classmethod
    @final
    @memoize
    async def get_flags(cls, options: Options) -> str:
        rsync_flags_parts = ['--recursive']
        identity_path = await (await cls.get_ssh(options)).get_identity_path(options)
        if identity_path is not None:
            rsync_flags_parts.append(fr'-e \"ssh -i {identity_path}\"')

        rsync_flags = ' '.join(rsync_flags_parts)

        log.debug(f'{cls.__name__} {rsync_flags = }')

        return rsync_flags

    @staticmethod
    @abstractmethod
    async def get_src_path(options: Options) -> Path:
        raise NotImplementedError

    @classmethod
    @final
    @memoize
    async def get_src_uri(cls, options: Options) -> Uri:
        src_uri = Uri(f'{(await cls.get_src_path(options))}')

        log.debug(f'{cls.__name__} {src_uri = }')

        return src_uri
