from dataclasses import dataclass, replace
from logging import getLogger
from pathlib import Path

from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenUniversal(Handler):

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        universal_path = options.universal_path

        if universal_path is None:
            universal_path = Path.home()

        universal_path = options.resolve_path(universal_path)

        return replace(options, universal_path=universal_path)

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        # TODO py38 debug string
        log.debug(f'universal_path: {options.universal_path}')

        assert options.universal_path is not None, 'universal_path cannot be None'
