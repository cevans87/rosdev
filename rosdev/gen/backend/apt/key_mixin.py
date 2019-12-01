from abc import ABC
from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.key_mixin_base import GenBackendAptKeyMixinBase
from rosdev.gen.backend.apt.local_key import GenBackendAptLocalKey
from rosdev.util.options import Options

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendAptKeyMixin(GenBackendAptKeyMixinBase, ABC):

    @classmethod
    @memoize
    async def main(cls, options: Options) -> None:
        if await cls.get_apt_key(options) != await GenBackendAptLocalKey.get_apt_key(options):
            await cls.execute(
                command=(
                    f'sudo sh -c "'
                    f"echo '{await GenBackendAptLocalKey.get_apt_key(options)}' | apt-key add -"
                    f'"'
                ),
                options=options,
            )
