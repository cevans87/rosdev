from dataclasses import dataclass, field, replace
from frozendict import frozendict
from logging import getLogger
from typing import Tuple, Type

from rosdev.gen.base import GenBase
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerGdbinit(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenBase,
    ))

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        docker_container_volumes = dict(options.docker_container_volumes)
        docker_container_volumes[options.docker_gdbinit_workspace_path] = (
            options.docker_gdbinit_container_path
        )
        docker_container_volumes = frozendict(options.docker_container_volumes)

        return replace(options, docker_container_volumes=docker_container_volumes)

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        # TODO py38 debug print
        log.debug(f'docker_gdbinit_container_path: {options.docker_gdbinit_container_path}')
        log.debug(f'docker_gdbinit_workspace_path: {options.docker_gdbinit_workspace_path}')

    @classmethod
    async def main(cls, options: Options) -> None:
        log.info(f'Creating docker_gdbinit')
        await cls.exec_workspace(
            f'mkdir -p {options.docker_gdbinit_workspace_path.parent}', err_ok=True
        )
        with open(str(options.docker_gdbinit_workspace_path), 'w') as f_out:
            # FIXME these are two common paths from ci.ro2.org builds. Find a way to
            #  programmatically find the paths, probably through jenkins.
            f_out.write(
                f'set substitute-path '
                f'/home/jenkins-agent/workspace/'
                f'packaging_{options.operating_system}/ws/src/ '
                f'{options.src_workspace_path}\n'
            )
            f_out.write(
                f'set substitute-path '
                f'/home/rosbuild/ci_scripts/ws/src/ '
                f'{options.src_workspace_path}\n'
            )

        log.info(f'Created docker_gdbinit')
