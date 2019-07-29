from dataclasses import dataclass, field, replace
from logging import getLogger
from typing import Tuple, Type

from rosdev.gen.ros.build_num import GenRosBuildNum
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.subprocess import exec


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerBuildContext(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenRosBuildNum,
    ))

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        docker_build_context_workspace_path = options.resolve_path(
            options.docker_build_context_workspace_path
        )

        return replace(
            options,
            docker_build_context_workspace_path=docker_build_context_workspace_path,
        )

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        # TODO py38 debug string
        log.debug(
            f'docker_build_context_workspace_path: {options.docker_build_context_workspace_path}'
        )

        assert (
            options.base_workspace_path in options.docker_build_context_workspace_path.parents
        ), 'docker_build_context_workspace_path must be a descendant of base_workspace_path'

    @classmethod
    async def main(cls, options: Options) -> None:
        log.info(f'Creating docker build context at {options.docker_build_context_workspace_path}')
        await exec(f'mkdir -p {options.docker_build_context_workspace_path}')
        log.info(f'Created docker build context at {options.docker_build_context_workspace_path}')
