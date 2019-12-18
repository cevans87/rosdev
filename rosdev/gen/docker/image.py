from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from typing import Tuple

from rosdev.gen.docker.dockerfile import GenDockerDockerfile
from rosdev.gen.docker.entrypoint_sh import GenDockerEntrypointSh
from rosdev.gen.docker.image_base import GenDockerImageBase
from rosdev.gen.host import GenHost
from rosdev.util.options import Options
from rosdev.util.path import Path


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerImage(GenDockerImageBase):

    @staticmethod
    async def execute(*, command: str, options: Options, err_ok=False) -> None:
        await GenHost.execute(
            command=(
                f'docker run --rm'
                f' -e {await GenDockerEntrypointSh.get_log_level_env_name(options)}'
                f' -v {Path.home()}:{Path.home()}'
                f' -v {Path.workspace()}:{Path.workspace()}'
                f' {await GenDockerImage.get_tag(options)}'
                f' {command}'
            ),
            err_ok=err_ok,
            options=options,
        )

    @staticmethod
    async def execute_and_get_lines(*, command: str, options: Options, err_ok=False) -> Tuple[str]:
        await GenDockerImage.run(options)
        return await GenHost.execute_and_get_lines(
            command=(
                f'docker run --rm'
                f' -v {Path.home()}:{Path.home()}'
                f' -v {Path.workspace()}:{Path.workspace()}'
                f' {await GenDockerImage.get_tag(options)}'
                f' {command}'
            ),
            err_ok=err_ok,
            options=options,
        )

    @staticmethod
    async def execute_and_get_line(*, command: str, options: Options, err_ok=False) -> str:
        return await GenHost.execute_and_get_line(
            command=(
                f'docker run --rm'
                f' -v {Path.home()}:{Path.home()}'
                f' -v {Path.workspace()}:{Path.workspace()}'
                f' {await GenDockerImage.get_tag(options)}'
                f' {command}'
            ),
            options=options,
            err_ok=err_ok,
        )

    @staticmethod
    @memoize
    async def get_ros_distro(options: Options) -> str:
        ros_distro = [
            v
            for env in (await GenDockerImage._get_inspect(options))['Config']['Env']
            for k, v in [env.split('=')]
            if k == 'ROS_DISTRO'
        ][0]

        log.debug(f'{GenDockerImage.__name__} {ros_distro = }')

        return ros_distro

    @staticmethod
    @memoize(db=True, keygen=lambda options: GenDockerImage.get_id(options), size=1)
    async def main(options: Options) -> None:
        log.info(f'Creating docker image {await GenDockerImageBase.get_tag(options)}.')
        await GenHost.execute(
            command=(
                f'docker image build {(await GenDockerDockerfile.get_path(options)).parent}'
                f'{" --pull" if options.docker_image_pull else ""}'
                f'{" --no-cache" if options.docker_image_replace else ""}'
                f' --tag {await GenDockerImageBase.get_tag(options)}'
            ),
            options=options,
        )
        log.info(f'Created docker image {await GenDockerImageBase.get_tag(options)}.')
