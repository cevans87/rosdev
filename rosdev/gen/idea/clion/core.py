from dataclasses import dataclass, field
from logging import getLogger
from typing import Tuple, Type

from rosdev.gen.idea.clion.base import GenIdeaClionBase
from rosdev.util.handler import Handler

log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaClionCore(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenIdeaClionBase,
    ))
