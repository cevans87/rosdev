from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.local_base import GenBackendLocalBase
from rosdev.gen.backend.home.runner_base import GenBackendHomeRunnerBase
from rosdev.gen.backend.workspace.mixin_base import GenBackendWorkspaceMixinBase
from rosdev.util.options import Options

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendWorkspaceRunnerBase(GenBackendWorkspaceMixinBase, GenBackendLocalBase):

    @staticmethod
    @final
    @memoize
    def get_home_base(options: Options) -> Type[GenBackendHomeRunnerBase]:
        home_base = GenBackendHomeRunnerBase

        log.debug(f'{__class__.__name__} {home_base = }')

        return home_base
