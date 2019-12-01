from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from textwrap import dedent

from rosdev.gen.host import GenHost
from rosdev.gen.rosdev.home import GenRosdevHome
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.path import Path

log = getLogger(__name__)


@dataclass(frozen=True)
class GenSshKnownHosts(Handler):

    @staticmethod
    @memoize
    async def get_mode(options: Options) -> int:
        mode = 0o644

        log.debug(f'{GenSshKnownHosts.__name__} {oct(mode) = }')

        return mode

    @staticmethod
    @memoize
    async def get_path(options: Options) -> Path:
        path = await GenRosdevHome.get_path(options) / 'ssh' / 'known_hosts'

        log.debug(f'{GenSshKnownHosts.__name__} {path = }')

        return path

    @staticmethod
    async def main(options: Options) -> None:
        if (
                (await GenSshKnownHosts.get_path(options)).exists() and
                (
                        (await GenSshKnownHosts.get_path(options)).stat().st_mode ==
                        await GenSshKnownHosts.get_mode(options)
                )
        ):
            return

        (await GenSshKnownHosts.get_path(options)).parent.mkdir(parents=True, exist_ok=True)
        (await GenSshKnownHosts.get_path(options)).touch(0o644)
