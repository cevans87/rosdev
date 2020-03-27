from asyncio import gather
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.entrypoint_sh.builder import GenBackendEntrypointShBuilder
from rosdev.gen.backend.entrypoint_sh.local import GenBackendEntrypointShLocal
from rosdev.gen.backend.entrypoint_sh.runner import GenBackendEntrypointShRunner
from rosdev.util.atools import memoize
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class AptUpdate(Handler):
    """Run apt update on all endpoints."""

    @staticmethod
    @memoize
    async def main(options: Options) -> None:
        command = f'apt update{" " + options.remainder if options.remainder else ""}'
        await gather(*[
            backend.execute(command=command, options=options, sudo=True) for backend in [
                GenBackendEntrypointShBuilder,
                GenBackendEntrypointShLocal,
                GenBackendEntrypointShRunner
            ]
        ])
