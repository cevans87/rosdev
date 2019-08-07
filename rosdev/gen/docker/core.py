from dataclasses import dataclass, field
from logging import getLogger
from typing import Tuple, Type

from rosdev.gen.core import GenCore
from rosdev.gen.docker.base import GenDockerBase
from rosdev.gen.docker.container import GenDockerContainer
from rosdev.gen.docker.environment import GenDockerEnvironment
from rosdev.gen.docker.image import GenDockerImage
from rosdev.gen.docker.install import GenDockerInstall
from rosdev.gen.docker.pam_environment import GenDockerPamEnvironment
from rosdev.gen.docker.ssh.start import GenDockerSshStart
from rosdev.util.handler import Handler


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerCore(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenCore,
        GenDockerBase,
        GenDockerContainer,
        GenDockerEnvironment,
        GenDockerImage,
        GenDockerInstall,
        GenDockerPamEnvironment,
        GenDockerSshStart,
    ))
