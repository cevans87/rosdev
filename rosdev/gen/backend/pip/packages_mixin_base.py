from abc import ABC
from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import FrozenSet

from rosdev.gen.backend.base import GenBackendBase
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendPipPackagesMixinBase(GenBackendBase, ABC):

    @classmethod
    @memoize
    async def get_pip_packages(cls, options: Options) -> FrozenSet[str]:
        pip_packages = frozenset(
            pip_package
            for line in await cls.execute_and_get_lines(
                command=f'python3 -m pip list', options=options
            )
            for (pip_package, _) in [line.split(' ', 1)]
        )

        log.debug(f'{cls.__name__} {pip_packages = }')

        return pip_packages
