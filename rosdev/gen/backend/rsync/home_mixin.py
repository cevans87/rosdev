from asyncio import gather
from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.builder_endpoint import GenBackendBuilder
from rosdev.gen.backend.local_endpoint import GenBackendLocal
from rosdev.gen.backend.runner_endpoint import GenBackendRunnerEndpoint
from rosdev.gen.backend.simulator_endpoint import GenBackendSimulatorEndpoint
from rosdev.gen.backend.endpoints import GenBackendEndpoints
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendSyncHome(Handler):

    @staticmethod
    @memoize
    async def main(options: Options) -> None:
        sync_home_futures = []
        for endpoint in (
                GenBackendBuilder,
                GenBackendRunnerEndpoint,
                GenBackendSimulatorEndpoint,
        ):
            sync_home_futures.append(
                # FIXME finish this command
                GenBackendLocal.execute(
                    command=f'rsync foo {await endpoint.get_ssh_uri(options)}',
                    options=options,
                )
            )
        await gather(*sync_home_futures)
