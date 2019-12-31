from asyncio import gather
from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.packages.mixin_base import GenBackendAptPackagesMixinBase
from rosdev.gen.backend.builder import GenBackendBuilder
from rosdev.gen.backend.local import GenBackendLocal
from rosdev.gen.backend.runner import GenBackendRunner
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class Install(Handler):
    """Run rosdep install on all endpoints."""

    @staticmethod
    @memoize
    async def main(options: Options) -> None:
        await GenBackendAptPackagesMixinBase.get_apt_packages.memoize.reset()
        await gather(
            *[
                backend.get_ssh(options).execute(
                    command=(
                        f'rosdep install'
                        f'{" " + " ".join(options.remainder) if options.remainder else ""}'
                    ),
                    options=options,
                )
                for backend in [GenBackendBuilder, GenBackendLocal, GenBackendRunner]
            ]
        )
