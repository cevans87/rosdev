from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from uuid import UUID, uuid4

from rosdev.util.handler import Handler


log = getLogger(__name__)


@memoize
@dataclass(frozen=True)
class Uuid(Handler):

    @property
    @memoize
    def uuid(self) -> UUID:
        return self.options.uuid if self.options.uuid is not None else uuid4()

    async def _main(self) -> None:
        pass
