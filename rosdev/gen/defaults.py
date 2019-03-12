from atools import memoize
from dataclasses import dataclass

from rosdev.util.handler import Handler


@memoize
@dataclass(frozen=True)
class Defaults(Handler):
    architecture = 'amd64'
    architectures = [architecture]
    asan = False
    bad_release = 'latest'
    clean = False
    debug = False
    gdbserver_port = 1337
    good_release = 'crystal'
    log_level = 'INFO'
    release = 'latest'
    releases = [release]

    @memoize
    async def _main(self) -> None:
        pass
