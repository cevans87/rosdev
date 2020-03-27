from dataclasses import dataclass
from logging import getLogger
from typing import final, Type

from rosdev.gen.backend.rsync.runner_base import GenBackendRsyncRunnerBase
from rosdev.gen.backend.rsync.workspace.mixin_base import GenBackendRsyncWorkspaceMixinBase
from rosdev.gen.backend.workspace.runner_base import GenBackendWorkspaceRunnerBase
from rosdev.util.atools import memoize
from rosdev.util.options import Options

log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRsyncWorkspaceRunnerBase(
    GenBackendRsyncWorkspaceMixinBase, GenBackendRsyncRunnerBase
):

    @classmethod
    @final
    @memoize
    def get_workspace_base(cls, options: Options) -> Type[GenBackendWorkspaceRunnerBase]:
        workspace_base = GenBackendWorkspaceRunnerBase

        log.debug(f'{cls.__name__} {workspace_base = }')

        return workspace_base
