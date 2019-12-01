from abc import ABC
from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.pip.local_packages import GenBackendPipLocalPackages
from rosdev.gen.backend.pip.packages_mixin_base import GenBackendPipPackagesMixinBase
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendPipPackagesMixin(GenBackendPipPackagesMixinBase, ABC):

    @classmethod
    @memoize
    async def main(cls, options: Options) -> None:
        if (
                await GenBackendPipLocalPackages.get_pip_packages(options) -
                await cls.get_pip_packages(options)
        ):
            await cls.execute(
                command=(
                    f' python3 -m pip install '
                    f' {" ".join(await GenBackendPipLocalPackages.get_pip_packages(options))}'
                ),
                options=options,
            )
