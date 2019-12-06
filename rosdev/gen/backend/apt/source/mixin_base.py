from abc import ABC
from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.backend.mixin_base import GenBackendMixinBase
from rosdev.util.options import Options
from rosdev.util.path import Path


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBackendAptSourceMixinBase(GenBackendMixinBase, ABC):

    @classmethod
    @memoize
    async def get_apt_source(cls, options: Options) -> str:
        apt_source = '\n'.join(
            await (await cls.get_ssh(options)).execute_and_get_lines(
                command=f'cat {await cls.get_apt_source_path(options)} 2> /dev/null',
                err_ok=True,
                options=options,
            )
        )

        log.debug(f'{cls.__name__} {apt_source = }')

        return apt_source

    @staticmethod
    @memoize
    async def get_apt_source_path(options: Options) -> Path:
        if options.release in {'kinetic', 'melodic'}:
            apt_source_path = Path('/etc/apt/sources.list.d/ros1-latest.list')
        else:
            apt_source_path = Path('/etc/apt/sources.list.d/ros2-latest.list')

        log.debug(f'{GenBackendMixinBase.__name__} {apt_source_path = }')

        return apt_source_path
