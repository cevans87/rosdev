from abc import ABC
from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.source.local_base import GenBackendAptSourceLocalBase
from rosdev.gen.backend.apt.source.mixin_base import GenBackendAptSourceMixinBase
from rosdev.util.options import Options
from rosdev.util.path import Path


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendAptSourceMixin(GenBackendAptSourceMixinBase, ABC):

    @classmethod
    @memoize(
        db=True,
        keygen=lambda cls, options: (
                cls.__name__,
                cls.get_ssh(options).get_uri(options),
                GenBackendAptSourceLocalBase.get_apt_source(options),
        )
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
            await cls.get_apt_source.memoize.reset_call(cls, options)
