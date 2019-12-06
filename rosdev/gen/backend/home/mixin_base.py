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
        path = Path(
            await (
                await cls.get_ssh(options)
            ).execute_and_get_line(
                command='echo $HOME', options=options
            )
        )

        log.debug(f'{cls.__name__} {path = }')

        return path
