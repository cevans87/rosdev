from asyncio import gather
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.packages.mixin_base import GenBackendAptPackagesMixinBase
from rosdev.gen.backend.entrypoint_sh.builder import GenBackendEntrypointShBuilder
from rosdev.gen.backend.entrypoint_sh.local import GenBackendEntrypointShLocal
from rosdev.gen.backend.entrypoint_sh.runner import GenBackendEntrypointShRunner
from rosdev.gen.backend.pip.packages.mixin_base import GenBackendPipPackagesMixinBase
from rosdev.util.atools import memoize
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class RosdepInstall(Handler):
    """Run apt install on all endpoints."""

    @staticmethod
    @memoize
    async def main(options: Options) -> None:
        # Running apt install may change what's installed on the backends, so we invalidate.
        await gather(*[
            backend.get_apt_packages.memoize.reset() for backend in [
                GenBackendAptPackagesMixinBase,
                GenBackendPipPackagesMixinBase
            ]
        ])

        command = f'apt install{" " + options.remainder if options.remainder else ""}'
        await gather(*[
            backend.execute(command=command, options=options, sudo=True) for backend in [
                GenBackendEntrypointShBuilder,
                GenBackendEntrypointShLocal,
                GenBackendEntrypointShRunner
            ]
        ])
