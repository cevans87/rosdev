from abc import ABC
from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import final

from rosdev.gen.backend.store.mixin_base import GenBackendStoreMixinBase
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendStoreMixin(GenBackendStoreMixinBase, ABC):

    @classmethod
    @final
    @memoize(
        db=True, keygen=lambda cls, options: (cls.__name__, cls.get_ssh(options).get_uri(options))
    )
    async def main(cls, options: Options) -> None:
        if await cls.is_local(options):
            return

        await cls.get_ssh(options).execute(
            command=(
                f'mkdir -p {(await cls.get_path(options)).resolve()} &&'
                f' mkdir -p {(await cls.get_path(options)).parent} &&'
                f' rm -rf {await cls.get_path(options)} &&'
                f' ln --force --symbolic --relative '
                f' {(await cls.get_path(options)).resolve()} {await cls.get_path(options)}'
            ),
            options=options,
        )
