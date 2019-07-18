from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.util.handler import Handler


log = getLogger(__name__)


@memoize
@dataclass(frozen=True)
class Workspace(Handler):

    async def _main(self) -> None:
        # TODO generate a workspace. keeping a ~/.rosdev/workspace file might be best.
        pass
