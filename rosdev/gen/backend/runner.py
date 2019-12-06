from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.key.runner import GenBackendAptKeyRunner
from rosdev.gen.backend.apt.packages.runner import GenBackendAptPackagesRunner
from rosdev.gen.backend.apt.source.runner import GenBackendAptSourceRunner
from rosdev.gen.backend.runner_base import GenBackendRunnerBase
from rosdev.gen.backend.install.runner import GenBackendInstallRunner
from rosdev.gen.backend.mixin import GenBackendMixin
from rosdev.gen.backend.pip.packages.runner import GenBackendPipPackagesRunner
from rosdev.gen.backend.rsync.runner import GenBackendRsyncRunner
from rosdev.gen.backend.rosdev.home.runner import GenBackendRosdevHomeRunner
from rosdev.gen.backend.rosdev.workspace.runner import GenBackendRosdevWorkspaceRunner


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendRunner(GenBackendRunnerBase, GenBackendMixin):
    pass
