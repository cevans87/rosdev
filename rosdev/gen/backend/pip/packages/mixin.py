from abc import ABC
from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.pip.packages.local_base import GenBackendPipPackagesLocalBase
from rosdev.gen.backend.pip.packages.mixin_base import GenBackendPipPackagesMixinBase
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendPipPackagesMixin(GenBackendPipPackagesMixinBase, ABC):

    @classmethod
    @memoize(
        db=True,
        keygen=lambda cls, options: (
                cls.get_ssh(options).get_uri(options),
                GenBackendPipPackagesLocalBase.get_pip_packages(options),
        ),
        size=3,
    )
    async def main(cls, options: Options) -> None:
        if (
                await GenBackendPipPackagesLocalBase.get_pip_packages(options) -
                await cls.get_pip_packages(options)
        ):
            await cls.get_ssh(options).execute(
                command=(
                    f' python3 -m pip install '
                    f' {" ".join(await GenBackendPipPackagesLocalBase.get_pip_packages(options))}'
                ),
                options=options,
                sudo=True,
            )
            await cls.get_pip_packages.memoize.reset_call(cls, options)
