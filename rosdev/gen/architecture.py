from asyncio import gather
from dataclasses import dataclass, field, replace
from logging import getLogger
from platform import release
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
    async def _resolve_architecture(cls, options: Options) -> Options:
        architecture = options.architecture
        if architecture is None:
            architecture = release().rsplit('-', 1)[-1]

        return replace(options, architecture=architecture)

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        options = await cls._resolve_architecture(options)

        architecture_container_path = options.resolve_container_path(
            options.architecture_container_path
        )
        architecture_universal_path = options.resolve_universal_path(
            options.architecture_universal_path
        )
        architecture_workspace_path = options.resolve_workspace_path(
            options.architecture_workspace_path
        )

        return replace(
            options,
            architecture_container_path=architecture_container_path,
            architecture_universal_path=architecture_universal_path,
            architecture_workspace_path=architecture_workspace_path,
        )

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        # TODO py38 debug print
        log.debug(f'architecture: {options.architecture}')
        log.debug(f'architecture_container_path: {options.architecture_container_path}')
        log.debug(f'architecture_universal_path: {options.architecture_universal_path}')
        log.debug(f'architecture_workspace_path: {options.architecture_workspace_path}')

        valid_architectures = {'amd64', 'arm32v7', 'arm64v8'}
        assert (
                options.architecture in valid_architectures
        ), f'architecture must be in {valid_architectures}'

    @classmethod
    async def main(cls, options: Options) -> None:
        await gather(
            exec(f'mkdir -p {options.architecture_universal_path}'),
            exec(f'mkdir -p {options.architecture_workspace_path}'),
        )
