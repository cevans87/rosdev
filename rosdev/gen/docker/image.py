from atools import memoize
from dataclasses import dataclass
import json
from logging import getLogger
from typing import Tuple

from rosdev.gen.docker.dockerfile import GenDockerDockerfile
from rosdev.gen.docker.entrypoint_sh import GenDockerEntrypointSh
from rosdev.gen.home import GenHome
from rosdev.gen.host import GenHost
from rosdev.gen.workspace import GenWorkspace
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerImage(Handler):

    @classmethod
    @memoize
    async def get_tag(cls, options: Options) -> str:
        tag = f'rosdev:{options.release}_{options.architecture}'
        
        log.debug(f'{cls.__name__} {tag = }')

        return tag

    @classmethod
    @memoize
    async def get_id(cls, options: Options) -> str:
        lines = await GenHost.execute_shell_and_get_lines(
            command=f'docker image inspect {await cls.get_tag(options)} 2> /dev/null',
            options=options,
        )
        return json.loads('\n'.join(lines))[0]['Id']

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
                f' -e {await GenDockerEntrypointSh.get_log_level_env_name(options)}'
                f' -v {await GenHome.get_path(options)}:{await GenHome.get_path(options)}'
                f' -v {await GenWorkspace.get_path(options)}:{await GenWorkspace.get_path(options)}'
                f' {await cls.get_tag(options)}'
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
                f' -v {await GenHome.get_path(options)}:{await GenHome.get_path(options)}'
                f' -v {await GenWorkspace.get_path(options)}:{await GenWorkspace.get_path(options)}'
                f' {await cls.get_tag(options)}'
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
                f' -v {await GenHome.get_path(options)}:{await GenHome.get_path(options)}'
                f' -v {await GenWorkspace.get_path(options)}:{await GenWorkspace.get_path(options)}'
                f' {await cls.get_tag(options)}'
                f' {command}'
            ),
            options=options,
            err_ok=err_ok,
        )

    @classmethod
    async def main(cls, options: Options) -> None:
        log.info(f'Creating docker image {await cls.get_tag(options)}')
        await GenHost.execute(
            command=(
                f'docker image build {(await GenDockerDockerfile.get_path(options)).parent}'
                f'{" --pull" if options.docker_image_pull else ""}'
                f'{" --no-cache" if options.docker_image_replace else ""}'
                f' --tag {await cls.get_tag(options)}'
            ),
            options=options,
        )
        log.info(
            f'Created docker image {await cls.get_tag(options)}'
            f' from {await GenDockerDockerfile.get_from(options)}'
        )
