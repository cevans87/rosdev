from dataclasses import dataclass, field
import json
from logging import getLogger
from typing import Tuple, Type

from rosdev.gen.docker.base import GenDockerBase
from rosdev.gen.host import GenHost
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerImage(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenDockerBase,
        GenHost,
    ))

    @classmethod
    async def execute(
            cls,
            *,
            command: str,
            options: Options,
            err_ok=False,
    ) -> None:
        await GenHost.execute(
            command=(
                f'docker run --rm'
                f' -e {options.docker_entrypoint_sh_log_level_env_name}={log.getEffectiveLevel()}'
                f' -v {options.home_path}:{options.home_path}'
                f' -v {options.workspace_path}:{options.workspace_path}'
                f' {options.docker_image_tag}'
                f' {command}'
            ),
            err_ok=err_ok,
            options=options,
        )

    @classmethod
    async def execute_and_get_lines(
            cls,
            *,
            command: str,
            options: Options,
            err_ok=False,
    ) -> Tuple[str]:
        return await GenHost.execute_and_get_lines(
            command=(
                f'docker run --rm'
                f' -v {options.home_path}:{options.home_path}'
                f' -v {options.workspace_path}:{options.workspace_path}'
                f' {options.docker_image_tag}'
                f' {command}'
            ),
            err_ok=err_ok,
            options=options,
        )

    @classmethod
    async def execute_and_get_line(
            cls,
            *,
            command: str,
            options: Options,
            err_ok=False,
    ) -> str:
        return await GenHost.execute_and_get_line(
            command=(
                f'docker run --rm'
                f' -v {options.home_path}:{options.home_path}'
                f' -v {options.workspace_path}:{options.workspace_path}'
                f' {options.docker_image_tag}'
                f' {command}'
            ),
            options=options,
            err_ok=err_ok,
        )

    @classmethod
    async def get_id(cls, options: Options) -> str:
        lines = await GenHost.execute_shell_and_get_lines(
            command=f'docker image inspect {options.docker_image_tag} 2> /dev/null',
            options=options,
        )
        return json.loads('\n'.join(lines))[0]['Id']

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        log.debug(f'{options.docker_image_base_tag = }')
        log.debug(f'{options.docker_image_tag = }')
        log.debug(f'{options.docker_image_replace = }')

    @classmethod
    async def main(cls, options: Options) -> None:
        log.info(
            f'Creating docker image {options.docker_image_tag}'
            f' from {options.docker_image_base_tag}'
        )
        await GenHost.execute(
            command=(
                f'docker image build {options.docker_dockerfile_path.parent}'
                f'{" --pull" if options.docker_image_pull else ""}'
                f'{" --no-cache" if options.docker_image_replace else ""}'
                f' --tag {options.docker_image_tag}'
            ),
            options=options,
        )
        log.info(
            f'Created docker image {options.docker_image_tag}'
            f' from {options.docker_image_base_tag}'
        )
