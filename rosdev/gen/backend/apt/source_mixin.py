from abc import ABC
from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.local_source import GenBackendAptLocalSource
from rosdev.gen.backend.apt.source_mixin_base import GenBackendAptSourceMixinBase
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendAptSourceMixin(GenBackendAptSourceMixinBase, ABC):

    @classmethod
    @memoize
    async def main(cls, options: Options) -> None:
        if (
                await cls.get_apt_source(options) !=
                await GenBackendAptLocalSource.get_apt_source(options)
        ):
            await cls.execute(
                command=(
                    f'sudo sh -c "'
                    f"echo '{await GenBackendAptLocalSource.get_apt_source(options)}' >"
                    f' {await cls.get_apt_source_path(options)}'
                    f'"'
                ),
                options=options,
            )
