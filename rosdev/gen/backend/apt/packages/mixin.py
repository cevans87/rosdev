from abc import ABC
from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.packages.local_base import GenBackendAptPackagesLocalBase
from rosdev.gen.backend.apt.packages.mixin_base import GenBackendAptPackagesMixinBase
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendAptPackagesMixin(GenBackendAptPackagesMixinBase, ABC):

    @classmethod
    @memoize(
        db=True,
        keygen=lambda cls, options: (
            cls.__name__,
            cls.get_ssh(options).get_uri(options),
            GenBackendAptPackagesLocalBase.get_apt_packages(options),
        )
    )
    async def main(cls, options: Options) -> None:
        if (
                await GenBackendAptPackagesLocalBase.get_apt_packages(options) -
                await cls.get_apt_packages(options)
        ):
            await cls.get_ssh(options).execute(
                command=(
                    f'apt update &&'
                    f' apt install -y'
                    f' {" ".join(await GenBackendAptPackagesLocalBase.get_apt_packages(options))}'
                ),
                options=options,
                sudo=True,
            )
            await cls.get_apt_packages.memoize.reset_call(cls, options)
