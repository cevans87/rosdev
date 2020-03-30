from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.entrypoint_sh.local import GenBackendEntrypointShLocal
from rosdev.gen.backend.apt.packages.local_base import GenBackendAptPackagesLocalBase
from rosdev.gen.backend.apt.packages.mixin_base import GenBackendAptPackagesMixinBase
from rosdev.gen.backend.cwd.local_base import GenBackendCwdLocalBase
from rosdev.gen.backend.entrypoint_sh.local_base import GenBackendEntrypointShLocalBase
from rosdev.gen.backend.local import GenBackendLocal
from rosdev.gen.rosdep.update import GenRosdepUpdate
from rosdev.util.atools import memoize, memoize_db
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.path import Path


log = getLogger(__name__)


@dataclass(frozen=True)
class GenRosdepInstall(Handler):
    """Run rosdep install on local endpoint."""

    @staticmethod
    @memoize
    async def get_command(options) -> str:
        command = 'rosdep install'
        if options.handler_module == __name__ or 'rosdev.install' == __name__:
            command = ' '.join([command, options.remainder])
        elif (Path.workspace() / 'src').exists():
            command += ' --from-paths src --ignore-src -r -y'

        log.debug(f'{__class__.__name__} {command = }')

        return command

    @staticmethod
    @memoize_db(keygen=lambda options: None, size=1)
    async def _main_inner(options: Options) -> None:
        await GenBackendAptPackagesMixinBase.get_apt_packages.memoize.remove(
            GenBackendAptPackagesLocalBase, options
        )
        await GenBackendEntrypointShLocal.execute(
            command=await GenRosdepUpdate.get_command(options),
            options=options,
        )
        await GenBackendLocal.get_ssh(options).execute(
            command=await GenRosdepUpdate.get_command(options),
            environment=await GenBackendEntrypointShLocalBase.get_path_by_env_key(options),
            options=options,
            path=await GenBackendCwdLocalBase.get_path(options),
        )

    @staticmethod
    @memoize
    async def main(options: Options) -> None:
        if options.handler_module == __name__ or 'rosdev.install' == __name__:
            GenRosdepInstall._main_inner.memoize.reset()

        await GenRosdepInstall._main_inner(options)
