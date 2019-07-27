import asyncio
from dataclasses import dataclass, field, replace
import docker
from logging import getLogger
from typing import Tuple, Type

from rosdev.gen.architecture import GenArchitecture
from rosdev.gen.docker.build.dockerfile import GenDockerBuildDockerfile
from rosdev.gen.ros.overlay.setup_bash import GenRosOverlaySetupBash
from rosdev.gen.ros.underlay.setup_bash import GenRosUnderlaySetupBash
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerImage(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenArchitecture,
        GenDockerBuildDockerfile,
        GenRosOverlaySetupBash,
        GenRosUnderlaySetupBash,
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
            if (options.ros_release != 'latest') or (cls.get_profile(options) != 'amd64'):
                docker_image_base_tag = (
                    f'{cls.get_profile(options)}/ros:{options.ros_release}-{options.flavor}'
                )
            else:
                docker_image_base_tag = 'osrf/ros2:nightly'
                
        docker_image_tag = options.docker_image_tag
        if docker_image_tag is None:
            docker_image_tag = f'{docker_image_base_tag}-dev'

        return replace(
            options,
            docker_image_base_tag=docker_image_base_tag,
            docker_image_tag=docker_image_tag,
        )

    @classmethod
    async def main(cls, options: Options) -> None:
        def main_internal() -> None:
            log.info(
                f'Creating docker image {options.docker_image_tag} '
                f'from {options.docker_image_base_tag}'
            )
            client = docker.client.from_env()
            client.images.build(
                path=str(options.docker_build_context_workspace_path),
                pull=options.pull_docker_image,
                rm=True,
                tag=options.docker_image_tag,
            )
            log.info(
                f'Created docker image {options.docker_image_tag} '
                f'from {options.docker_image_base_tag}'
            )

        await asyncio.get_event_loop().run_in_executor(None, main_internal)
