from abc import ABC
from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.key.mixin_base import GenBackendAptKeyMixinBase
from rosdev.gen.backend.apt.key.local_base import GenBackendAptKeyLocalBase
from rosdev.util.options import Options
from rosdev.util.path import Path

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendAptKeyMixin(GenBackendAptKeyMixinBase, ABC):

    @classmethod
    @memoize(
        db=Path.db(),
        keygen=lambda cls, options: (
                cls.__name__,
                cls.get_ssh(options).get_uri(options),
                GenBackendAptKeyLocalBase.get_apt_key(options),
        )
    )
    async def main(cls, options: Options) -> None:
        if await cls.get_apt_key(options) != await GenBackendAptKeyLocalBase.get_apt_key(options):
            await cls.get_ssh(options).execute(
                command=(
                    f"echo '{await GenBackendAptKeyLocalBase.get_apt_key(options)}' | apt-key add -"
                ),
                options=options,
                sudo=True,
            )
            await cls.get_apt_key.memoize.reset_call(cls, options)
