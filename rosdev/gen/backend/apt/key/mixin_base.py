from abc import ABC
from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.mixin_base import GenBackendMixinBase
from rosdev.util.options import Options
from rosdev.util.path import Path

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendAptKeyMixinBase(GenBackendMixinBase, ABC):

    @classmethod
    @memoize(
        db=Path.db(),
        keygen=lambda cls, options: (cls.__name__, cls.get_ssh(options).get_uri(options))
    )
    async def get_apt_key(cls, options: Options) -> str:
        apt_key = '\n'.join(
            await cls.get_ssh(options).execute_and_get_lines(
                command='apt-key exportall',
                options=options,
            )
        )

        log.debug(f'{cls.__name__} {apt_key = }')

        return apt_key
