from dataclasses import dataclass, field
from logging import getLogger
from typing import Tuple, Type

from rosdev.gen.architecture import GenArchitecture
from rosdev.gen.base import GenBase
from rosdev.gen.ros.build.num import GenRosBuildNum
from rosdev.gen.ros.release import GenRosRelease
from rosdev.util.handler import Handler


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerDockerfile(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenArchitecture,
        GenBase,
        GenRosBuildNum,
        GenRosRelease,
    ))
