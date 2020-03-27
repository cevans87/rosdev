from abc import ABC, abstractmethod
from dataclasses import dataclass
from logging import getLogger
from typing import Optional

from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.path import Path
from rosdev.util.uri import Uri


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendSshMixinBase(Handler, ABC):

    @staticmethod
    @abstractmethod
    async def get_identity_path(options: Options) -> Optional[Path]:
        raise NotImplementedError

    @staticmethod
    @abstractmethod
    async def get_uri(options: Options) -> Uri:
        raise NotImplementedError
