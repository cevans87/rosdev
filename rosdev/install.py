from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.rosdep.install import GenRosdepInstall
from rosdev.util.handler import Handler


log = getLogger(__name__)


@dataclass(frozen=True)
class Install(Handler):
    pass
