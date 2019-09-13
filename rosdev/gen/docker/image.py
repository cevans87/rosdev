from dataclasses import dataclass, field, replace
from logging import getLogger
from typing import Tuple, Type

from rosdev.gen.docker.base import GenDockerBase
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerImage(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenDockerBase,
    ))

    @classmethod
    def get_profile(cls, options: Options) -> str:
        if options.flavor in {'desktop', 'desktop-full'}:
            return 'osrf'
        else:
            return options.architecture

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        docker_image_base_tag = options.docker_image_base_tag
        if docker_image_base_tag is None:
            docker_image_base_tag = (
                f'{cls.get_profile(options)}/ros:{options.release}-{options.flavor}'
            )

        return replace(
            options,
            docker_image_base_tag=docker_image_base_tag,
        )

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        # TODO py38 debug print
        log.debug(f'docker_image_base_tag: {options.docker_image_base_tag}')
        log.debug(f'docker_image_tag: {options.docker_image_tag}')

    @classmethod
    async def main(cls, options: Options) -> None:
        log.info(
            f'Creating docker image {options.docker_image_tag} '
            f'from {options.docker_image_base_tag}'
        )
        await cls.exec_workspace(
            f'docker image build {options.docker_dockerfile_workspace_path.parent} '
            f'--tag {options.docker_image_tag}'
            f'{"--pull" if options.pull_docker_image else ""}'
        )
        log.info(
            f'Created docker image {options.docker_image_tag} '
            f'from {options.docker_image_base_tag}'
        )
