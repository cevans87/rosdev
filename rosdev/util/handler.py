from abc import ABC, abstractmethod
from dataclasses import dataclass
from logging import getLogger
from typing import Generator

from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class Handler(ABC):
    _options: Options

    def __await__(self) -> Generator[None, None, None]:
        return self._main().__await__()

    @property
    def options(self) -> Options:
        return self._options

    @abstractmethod
    async def _main(self) -> None:
        raise NotImplemented()
