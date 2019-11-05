from dataclasses import dataclass
from logging import getLogger

from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenArchitecture(Handler):

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        log.debug(f'{options.architecture = }')

        valid_architectures = {'amd64', 'arm32v7', 'arm64v8'}
        assert (
                options.architecture in valid_architectures
        ), f'Must be on of {valid_architectures}: {options.architecture = }'
