from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.builder_base import GenBackendBuilderBase
from rosdev.gen.backend.rsync.user_mixin_base import GenBackendRsyncUserMixinBase
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRsyncBuilderUser(GenBackendRsyncUserMixinBase, GenBackendBuilderBase):

    @staticmethod
    @memoize
    async def main(options: Options) -> None:
        pass
