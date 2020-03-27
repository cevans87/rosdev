from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.entrypoint_sh.local_base import GenBackendEntrypointShLocalBase
from rosdev.gen.backend.local import GenBackendLocal
from rosdev.gen.backend.cwd.local_base import GenBackendCwdLocalBase
from rosdev.util.atools import memoize, memoize_db
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenRosdepUpdate(Handler):
    """Run rosdep update on local endpoint."""

    @staticmethod
    @memoize
    async def get_command(options) -> str:
        command = 'rosdep update'
        if options.handler_module == __name__:
            command = ' '.join([command, options.remainder])

        log.debug(f'{__class__.__name__} {command = }')

        return command

    @staticmethod
    @memoize_db(keygen=lambda options: None)
    async def main(options: Options) -> None:
        await GenBackendLocal.get_ssh(options).execute(
            command=await GenRosdepUpdate.get_command(options),
            environment=await GenBackendEntrypointShLocalBase.get_path_by_env_key(options),
            options=options,
            path=await GenBackendCwdLocalBase.get_path(options),
        )
