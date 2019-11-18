from atools import memoize
from dataclasses import dataclass
import json
from logging import getLogger
from typing import Mapping, Tuple

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

    @staticmethod
    @memoize
    async def get_tag(options: Options) -> str:
        tag = f'rosdev:{options.release}_{options.architecture}'
        
        log.debug(f'{GenDockerImage.__name__} {tag = }')

        return tag

    @staticmethod
    @memoize
    async def get_id(options: Options) -> str:
        # noinspection PyShadowingBuiltins
        id = (await GenDockerImage.get_inspect(options)).get('Id', '')
        
        log.debug(f'{GenDockerImage.__name__} {id = }')
        
        return id

    @staticmethod
    @memoize
    async def get_inspect(options: Options) -> Mapping:
        lines = await GenHost.execute_shell_and_get_lines(
            command=f'docker image inspect {await GenDockerImage.get_tag(options)} 2> /dev/null',
            options=options,
            err_ok=True,
        )
        inspect = array[0] if (array := json.loads('\n'.join(lines))) else {}

        log.debug(f'{GenDockerImage.__name__} {inspect = }')
        
        return inspect

    @staticmethod
    @memoize
    async def get_ros_distro(options: Options) -> str:
        ros_distro = [
            v
            for env in (await GenDockerImage.get_inspect(options))['Config']['Env']
            for k, v in [env.split('=')]
            if k == 'ROS_DISTRO'
        ][0]

        log.debug(f'{GenDockerImage.__name__} {ros_distro = }')
        
        return ros_distro

    @staticmethod
    async def execute(*, command: str, options: Options, err_ok=False) -> None:
        await GenHost.execute(
            command=(
                f'docker run --rm'
                f' -e {await GenDockerEntrypointSh.get_log_level_env_name(options)}'
                f' -v {await GenHome.get_path(options)}:{await GenHome.get_path(options)}'
                f' -v {await GenWorkspace.get_path(options)}:{await GenWorkspace.get_path(options)}'
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
                f' -v {await GenHome.get_path(options)}:{await GenHome.get_path(options)}'
                f' -v {await GenWorkspace.get_path(options)}:{await GenWorkspace.get_path(options)}'
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
                f' -v {await GenHome.get_path(options)}:{await GenHome.get_path(options)}'
                f' -v {await GenWorkspace.get_path(options)}:{await GenWorkspace.get_path(options)}'
                f' {await GenDockerImage.get_tag(options)}'
                f' {command}'
            ),
            options=options,
            err_ok=err_ok,
        )

    @classmethod
    async def main(cls, options: Options) -> None:
        if not await GenDockerImage.get_id(options):
            log.info('Docker image does not exist.')
        elif (not options.docker_image_pull) and (not options.docker_image_replace):
            log.debug('Docker image is already built.')
            return

        log.info(f'Creating docker image {await cls.get_tag(options)}.')
        await GenHost.execute(
            command=(
                f'docker image build {(await GenDockerDockerfile.get_path(options)).parent}'
                f'{" --pull" if options.docker_image_pull else ""}'
                f'{" --no-cache" if options.docker_image_replace else ""}'
                f' --tag {await cls.get_tag(options)}'
            ),
            options=options,
        )
        GenDockerImage.get_id.memoize.reset()
        GenDockerImage.get_inspect.memoize.reset()
        log.info(f'Created docker image {await cls.get_tag(options)}.')
