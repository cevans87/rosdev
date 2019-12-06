from abc import ABC, abstractmethod
from dataclasses import dataclass
from logging import getLogger
from typing import Type

from rosdev.gen.backend.rosdev.home.mixin_base import GenBackendRosdevHomeMixinBase
from rosdev.gen.backend.rsync.home.mixin import GenBackendRsyncHomeMixin
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRosdevHomeMixin(GenBackendRosdevHomeMixinBase, ABC):

    @staticmethod
    @abstractmethod
    async def get_rsync_home(options: Options) -> Type[GenBackendRsyncHomeMixin]:
        raise NotImplementedError
