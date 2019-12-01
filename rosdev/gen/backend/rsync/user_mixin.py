from abc import ABC
from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.rsync.user_mixin_base import GenBackendRsyncUserMixinBase
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRsyncUserMixin(GenBackendRsyncUserMixinBase, ABC):

    @staticmethod
    @memoize
    async def main(options: Options) -> None:
        pass
