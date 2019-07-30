from dataclasses import dataclass, field, replace
from logging import getLogger
from typing import Tuple, Type

from rosdev.gen.home import GenHome
from rosdev.util.handler import Handler
from rosdev.util.lookup import get_operating_system
from rosdev.util.options import Options
from rosdev.util.subprocess import exec


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerGdbinit(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenHome,
    ))

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        docker_gdbinit_container_path = options.resolve_path(options.docker_gdbinit_container_path)

        docker_gdbinit_workspace_path = options.resolve_path(options.docker_gdbinit_workspace_path)

        return replace(
            options,
            docker_gdbinit_container_path=docker_gdbinit_container_path,
            docker_gdbinit_workspace_path=docker_gdbinit_workspace_path
        )

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        # TODO py38 debug print
        log.debug(f'docker_gdbinit_container_path: {options.docker_gdbinit_container_path}')
        log.debug(f'docker_gdbinit_workspace_path: {options.docker_gdbinit_workspace_path}')

    @classmethod
    async def main(cls, options: Options) -> None:
        log.info(f'Creating docker_gdbinit')
        await exec(f'mkdir -p {options.docker_gdbinit_workspace_path.parent}', err_ok=True)
        with open(str(options.docker_gdbinit_workspace_path), 'w') as f_out:
            # FIXME these are two common paths from ci.ro2.org builds. Find a way to
            #  programmatically find the paths, probably through jenkins.
            f_out.write(
                f'set substitute-path '
                f'/home/jenkins-agent/workspace/'
                f'packaging_{get_operating_system(options.architecture)}/ws/src/ '
                f'{options.ros_src_workspace_path}\n'
            )
            f_out.write(
                f'set substitute-path '
                f'/home/rosbuild/ci_scripts/ws/src/ '
                f'{options.ros_src_workspace_path}\n'
            )
        #await exec(
        #    f'docker container exec {options.docker_container_name} '
        #    f'ln -s {options.docker_gdbinit_container_path} '
        #    f'{options.home_container_path}/.gdbinit'
        #)

        log.info(f'Created docker_gdbinit')
