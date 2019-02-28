from __future__ import annotations
from abc import ABC, abstractmethod
from typing import Generator


class Handler(ABC):

    def __await__(self) -> Generator[None, None, None]:
        return self._run().__await__()

    @abstractmethod
    async def _run(self) -> None:
        raise NotImplemented()
