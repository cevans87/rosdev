from dataclasses import dataclass, field
from logging import getLogger
from typing import Tuple, Type

from rosdev.gen.docker.core import GenDockerCore
from rosdev.gen.idea.base import GenIdeaBase
from rosdev.gen.idea.ide.start import GenIdeaIdeStart
from rosdev.util.handler import Handler


log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaCore(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenDockerCore,
        GenIdeaBase,
        GenIdeaIdeStart,
    ))
