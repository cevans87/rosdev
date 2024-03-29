from abc import ABC
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.source.local_base import GenBackendAptSourceLocalBase
from rosdev.gen.backend.apt.source.mixin_base import GenBackendAptSourceMixinBase
from rosdev.util.atools import memoize_db
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendAptSourceMixin(GenBackendAptSourceMixinBase, ABC):

    @classmethod
    @memoize_db(
        keygen=lambda cls, options: (
                cls.get_ssh(options).get_uri(options),
                GenBackendAptSourceLocalBase.get_apt_source(options),
        ),
        size=3,
    )
    async def main(cls, options: Options) -> None:
        if (
                await cls.get_apt_source(options) !=
                await GenBackendAptSourceLocalBase.get_apt_source(options)
        ):
            await cls.get_ssh(options).execute(
                command=(
                    f"echo '{await GenBackendAptSourceLocalBase.get_apt_source(options)}' >"
                    f' {await cls.get_apt_source_path(options)}'
                ),
                options=options,
                sudo=True,
            )
            await cls.get_apt_source.memoize.remove(cls, options)
