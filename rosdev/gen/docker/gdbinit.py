from dataclasses import dataclass, field, replace
from frozendict import frozendict
from logging import getLogger
from typing import Tuple, Type

from rosdev.gen.base import GenBase
from rosdev.gen.docker.container import GenDockerContainer
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerGdbinit(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenBase,
        GenDockerContainer,
    ))

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        docker_container_volumes = dict(options.docker_container_volumes)
        docker_container_volumes[options.docker_gdbinit_path] = options.docker_gdbinit_path
        docker_container_volumes = frozendict(options.docker_container_volumes)

        return replace(options, docker_container_volumes=docker_container_volumes)

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        log.debug(f'{options.docker_gdbinit_symlink_container_path = }')
        log.debug(f'{options.docker_gdbinit_path = }')

    @classmethod
    async def main(cls, options: Options) -> None:
        log.info(f'Creating docker_gdbinit')
        cls.write_text(
            path=options.docker_gdbinit_path,
            # FIXME these are two common paths from ci.ro2.org builds. Find a way to
            #  programmatically find the paths, probably through jenkins.
            text=(
                f'set directories {options.src_symlink_path / "ros2"}'
                f'set substitute-path '
                f'/home/jenkins-agent/workspace/'
                f'packaging_{options.operating_system}/ws/src/ '
                f'{options.src_symlink_path}\n'
                f'set substitute-path '
                f'/home/rosbuild/ci_scripts/ws/src/ '
                f'{options.src_symlink_path}\n'
            )
        )
        await cls.execute_container(
            command=(
                f'ln -f -s '
                f'{options.docker_gdbinit_path} '
                f'{options.docker_gdbinit_symlink_container_path}'
            ),
            options=options,
        )

        log.info(f'Created docker_gdbinit')
