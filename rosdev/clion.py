from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.clion.ide import Ide
from rosdev.gen.clion.toolchain import Toolchain
from rosdev.util.handler import Handler


log = getLogger(__name__)


@memoize
@dataclass(frozen=True)
class Clion(Handler):

    @memoize
    async def _main(self) -> None:
        await Toolchain(self.options)
        await Ide(self.options)
