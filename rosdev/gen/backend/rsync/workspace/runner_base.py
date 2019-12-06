from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.rsync.runner_base import GenBackendRsyncRunnerBase
from rosdev.gen.backend.rsync.workspace.mixin_base import GenBackendRsyncWorkspaceMixinBase
from rosdev.gen.backend.workspace.runner_base import GenBackendWorkspaceRunnerBase
from rosdev.gen.backend.workspace.mixin_base import GenBackendWorkspaceMixinBase
from rosdev.util.options import Options

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRsyncWorkspaceRunnerBase(
    GenBackendRsyncWorkspaceMixinBase, GenBackendRsyncRunnerBase
):

    @classmethod
    @final
    @memoize
    async def get_workspace(cls, options: Options) -> Type[GenBackendWorkspaceMixinBase]:
        home = GenBackendWorkspaceRunnerBase

        log.debug(f'{cls.__name__} {home = }')

        return home
