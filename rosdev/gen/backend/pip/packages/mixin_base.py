from abc import ABC
from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import FrozenSet

from rosdev.gen.backend.mixin_base import GenBackendMixinBase
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendPipPackagesMixinBase(GenBackendMixinBase, ABC):

    @classmethod
    @memoize(
        db=True, keygen=lambda cls, options: (cls.__name__, cls.get_ssh(options).get_uri(options))
    )
    async def get_pip_packages(cls, options: Options) -> FrozenSet[str]:
        # FIXME use pip freeze to get packages in correct format.
        pip_packages = frozenset(
            pip_package
            for line in await cls.get_ssh(options).execute_and_get_lines(
                command=f'python3 -m pip list', options=options
            )
            for (pip_package, _) in [line.split(' ', 1)]
        )

        log.debug(f'{cls.__name__} {pip_packages = }')

        return pip_packages
