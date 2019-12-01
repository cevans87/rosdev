from abc import ABC
from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.local_packages import GenBackendAptLocalPackages
from rosdev.gen.backend.apt.packages_mixin_base import GenBackendAptPackagesMixinBase
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendAptPackagesMixin(GenBackendAptPackagesMixinBase, ABC):

    @classmethod
    @memoize
    async def main(cls, options: Options) -> None:
        if (
                await GenBackendAptLocalPackages.get_apt_packages(options) -
                await cls.get_apt_packages(options)
        ):
            await cls.execute(
                command=(
                    f'sudo sh -c "'
                    f'apt update &&'
                    f' apt install -y'
                    f' {" ".join(await GenBackendAptLocalPackages.get_apt_packages(options))}'
                    f'"'
                ),
                options=options,
            )
