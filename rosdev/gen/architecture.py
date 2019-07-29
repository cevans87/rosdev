from asyncio import gather
from dataclasses import dataclass, field, replace
from logging import getLogger
from platform import machine
from typing import Tuple, Type

from rosdev.gen.rosdev import GenRosdev
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.subprocess import exec


log = getLogger(__name__)


@dataclass(frozen=True)
class GenArchitecture(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenRosdev,
    ))

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        architecture = options.architecture
        if architecture is None:
            architecture = {
                'x86_64': 'amd64',
                'arm': 'arm32v7',
                'aarch64': 'arm64v8',
            }[machine()]

        return replace(options, architecture=architecture)

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        # TODO py38 debug print
        log.debug(f'architecture: {options.architecture}')

        valid_architectures = {'amd64', 'arm32v7', 'arm64v8'}
        assert (
                options.architecture in valid_architectures
        ), f'architecture must be in {valid_architectures}'
