from abc import ABC
from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import final

from rosdev.gen.backend.mixin_base import GenBackendMixinBase
from rosdev.util.options import Options
from rosdev.util.path import Path


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendHomeMixinBase(GenBackendMixinBase, ABC):

    @classmethod
    @final
    @memoize
    async def get_path(cls, options: Options) -> Path:
        path = Path(await cls._get_path_str(options))

        log.debug(f'{cls.__name__} {path = }')

        return path

    @classmethod
    @final
    @memoize(
        db=Path.db(),
        keygen=lambda cls, options: (f'{cls.__name__}', cls.get_ssh(options).get_uri(options))
    )
    async def _get_path_str(cls, options: Options) -> str:
        path_str = (
            await cls.get_ssh(options).execute_and_get_line(command='echo $HOME', options=options)
        )

        log.debug(f'{cls.__name__} {path_str = }')

        return path_str
