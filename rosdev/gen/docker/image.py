from dataclasses import dataclass, field, replace
import json
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
    async def get_id(cls, options: Options) -> str:
        lines = await cls.execute_shell_host_and_get_lines(
            command=f'docker image inspect {options.docker_image_tag} 2> /dev/null'
        )
        return json.loads('\n'.join(lines))[0]['Id']

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        docker_image_base_tag = options.docker_image_base_tag
        if docker_image_base_tag is None:
            if options.release == 'latest':
                assert options.architecture == 'amd64', f'Must be amd64: {options.architecture = }'
                docker_image_base_tag = 'osrf/ros2:nightly'
            else:
                docker_image_base_tag = f'{options.architecture}/ros:{options.release}'

        return replace(
            options,
            docker_image_base_tag=docker_image_base_tag,
        )

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        log.debug(f'{options.docker_image_base_tag = }')
        log.debug(f'{options.docker_image_tag = }')

    @classmethod
    async def main(cls, options: Options) -> None:
        log.info(
            f'Creating docker image {options.docker_image_tag} '
            f'from {options.docker_image_base_tag}'
        )
        await cls.execute_host(
            command=(
                f'docker image build {options.docker_dockerfile_path.parent} '
                f'--tag {options.docker_image_tag}'
                f'{"--pull" if options.pull_docker_image else ""}'
            )
        )
        log.info(
            f'Created docker image {options.docker_image_tag} '
            f'from {options.docker_image_base_tag}'
        )
