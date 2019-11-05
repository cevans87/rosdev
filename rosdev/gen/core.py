from dataclasses import dataclass, field
from logging import getLogger
from typing import Tuple, Type

from rosdev.gen.base import GenBase
from rosdev.gen.host import GenHost
from rosdev.gen.install import GenInstall
from rosdev.gen.src import GenSrc
from rosdev.util.handler import Handler

log = getLogger(__name__)


@dataclass(frozen=True)
class GenCore(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenBase,
        GenHost,
        GenInstall,
        GenSrc,
    ))
