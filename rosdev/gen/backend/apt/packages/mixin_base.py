from abc import ABC
from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import FrozenSet

from rosdev.gen.backend.mixin_base import GenBackendMixinBase
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendAptPackagesMixinBase(GenBackendMixinBase, ABC):

    @classmethod
    @memoize
    async def get_apt_packages(cls, options: Options) -> FrozenSet[str]:
        apt_packages = frozenset(
            await (await cls.get_ssh(options)).execute_and_get_lines(
                command=f'apt list --installed | grep -o "^[^/]*"',
                options=options,
            )
        )

        log.debug(f'{cls.__name__} {apt_packages = }')

        return apt_packages