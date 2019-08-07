from dataclasses import dataclass, field
from logging import getLogger
from typing import Tuple, Type

from rosdev.gen.architecture import GenArchitecture
from rosdev.gen.build_num import GenBuildNum
from rosdev.gen.container import GenContainer
from rosdev.gen.home import GenHome
from rosdev.gen.release import GenRelease
from rosdev.gen.workspace import GenWorkspace
from rosdev.gen.universal import GenUniversal
from rosdev.util.handler import Handler


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBase(Handler):
    
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenArchitecture,
        GenBuildNum,
        GenContainer,
        GenHome,
        GenRelease,
        GenWorkspace,
        GenUniversal,
    ))
