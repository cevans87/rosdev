from abc import ABC
from dataclasses import dataclass
from logging import getLogger
from typing import final

from rosdev.gen.backend.mixin_base import GenBackendMixinBase
from rosdev.util.atools import memoize_db
from rosdev.util.options import Options

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendAptKeyMixinBase(GenBackendMixinBase, ABC):

    @classmethod
    @final
    @memoize_db(keygen=lambda cls, options: cls.get_ssh(options).get_uri(options), size=3)
    async def get_apt_key(cls, options: Options) -> str:
        apt_key = '\n'.join(
            await cls.get_ssh(options).execute_and_get_lines(
                command='apt-key exportall',
                options=options,
            )
        )

        log.debug(f'{cls.__name__} {apt_key = }')

        return apt_key
