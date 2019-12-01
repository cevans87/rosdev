from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.apt.local_key import GenBackendAptLocalKey
from rosdev.gen.backend.apt.local_packages import GenBackendAptLocalPackages
from rosdev.gen.backend.apt.local_source import GenBackendAptLocalSource
from rosdev.gen.backend.local_base import GenBackendLocalBase


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendLocal(GenBackendLocalBase):
    pass
