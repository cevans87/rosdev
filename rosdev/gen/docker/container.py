from atools import memoize
from dataclasses import dataclass
from frozendict import frozendict
from logging import getLogger
from typing import Dict, Mapping, Tuple

from rosdev.gen.docker.container_base import GenDockerContainerBase
from rosdev.gen.docker.gdbinit import GenDockerGdbinit
from rosdev.gen.docker.entrypoint_sh import GenDockerEntrypointSh
from rosdev.gen.docker.image import GenDockerImage
from rosdev.gen.docker.ssh_base import GenDockerSshBase
from rosdev.gen.host import GenHost
from rosdev.gen.install import GenInstall
from rosdev.gen.src import GenSrc
from rosdev.gen.rosdev.home import GenRosdevHome
from rosdev.gen.workspace import GenWorkspace
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerContainer(GenDockerContainerBase):

    @staticmethod
    @memoize
    async def execute(
            command: str,
            options: Options,
            err_ok=False,
    ) -> None:
        await GenHost.execute(
            command=(
                f'docker exec'
                f' -e {await GenDockerEntrypointSh.get_log_level_env_name(options)}'
                f'={log.getEffectiveLevel()}'
                f' {await GenDockerEntrypointSh.get_environment_flags(options)}'
                f' {await GenDockerContainer.get_name(options)}'
                f' {await GenDockerEntrypointSh.get_container_path(options)}'
                f' {command}'
            ),
            err_ok=err_ok,
            options=options,
        )

    @staticmethod
    @memoize
    async def execute_and_get_lines(
            command: str,
            options: Options,
            err_ok=False,
    ) -> Tuple[str]:
        return await GenHost.execute_and_get_lines(
            command=(
                f'docker exec'
                f' {await GenDockerEntrypointSh.get_environment_flags(options)}'
                f' {await GenDockerContainer.get_name(options)}'
                f' {await GenDockerEntrypointSh.get_container_path(options)}'
                f' {command}'
            ),
            err_ok=err_ok,
            options=options,
        )

    @staticmethod
    @memoize
    async def execute_and_get_line(
            command: str,
            options: Options,
            err_ok=False,
    ) -> str:
        return await GenHost.execute_and_get_line(
            command=(
                f'docker exec'
                f' {await GenDockerEntrypointSh.get_environment_flags(options)}'
                f' {await GenDockerContainer.get_name(options)}'
                f' {await GenDockerEntrypointSh.get_container_path(options)}'
                f' {command}'
            ),
            options=options,
            err_ok=err_ok,
        )

    @staticmethod
    @memoize
    async def get_environment(options: Options) -> Mapping[str, str]:
        required_environment_keys = frozenset({
            'AMENT_PREFIX_PATH',
            'CMAKE_PREFIX_PATH',
            'COLCON_PREFIX_PATH',
            'LD_LIBRARY_PATH',
            'PATH',
            'PYTHONPATH',
            'ROS_DISTRO',
            'ROS_PYTHON_VERSION',
            'ROS_VERSION',
        })

        environment: Dict[str, str] = {}
        for line in await GenDockerContainer.execute_and_get_lines(command=f'env', options=options):
            try:
                k, v = line.split('=', 1)
            except ValueError:
                continue
            else:
                if k in required_environment_keys:
                    environment[k] = v
        # Building with raw cmake will fail to find certain ros libraries unless it has correct
        # path. AMENT_PREFIX_PATH will do the right thing. See
        # https://answers.ros.org/question/296462/compilation-error-building-against-binary-bouncy-could-not-find-fastrtps/
        if 'AMENT_PREFIX_PATH' in environment:
            environment['CMAKE_PREFIX_PATH'] = environment['AMENT_PREFIX_PATH']

        missing_environment_keys = required_environment_keys - environment.keys()
        if missing_environment_keys:
            log.warning(f'{missing_environment_keys = }')

        environment: Mapping[str, str] = frozendict(environment)

        log.debug(f'{GenDockerContainer.__name__} {environment = }')

        return environment

    @classmethod
    async def main(cls, options: Options) -> None:
        if not await GenDockerContainer.get_id(options):
            log.info('Docker container does not exist.')
        elif (
                options.docker_container_replace or
                (await GenDockerContainer.get_id(options) != await GenDockerImage.get_id(options))
        ):
            log.info('Replacing existing docker container.')
            await GenHost.execute(
                command=f'docker container rm -f {await GenDockerContainer.get_name(options)}',
                options=options,
            )
        elif not await GenDockerContainer.get_running(options):
            log.info('Restarting stopped docker container.')
            await GenHost.execute(
                command=f'docker start {await GenDockerContainer.get_name(options)}',
                options=options,
            )
            return
        else:
            log.debug('Docker container is already running.')
            return

        log.info(f'Creating docker container {await GenDockerContainer.get_name(options)}.')
        await GenHost.execute(
            command=(
                f'docker run'
                f' --detach'
                f' --expose 22'
                f' --hostname {await GenDockerContainer.get_name(options)}'
                f' --ipc host'
                f' --mount type=volume'
                f',dst={await GenInstall.get_container_path(options)}'
                f',volume-driver=local'
                f',volume-opt=type=none'
                f',volume-opt=o=bind'
                f',volume-opt=device={await GenInstall.get_home_path(options)}'
                f' --name {await GenDockerContainer.get_name(options)}'
                f' --privileged'
                f' --publish-all'
                f' --security-opt seccomp=unconfined'
                f' --volume {await GenDockerGdbinit.get_home_path(options)}'
                f':{await GenDockerGdbinit.get_container_path(options)}'
                f' --volume {await GenDockerSshBase.get_path(options)}'
                f':{await GenDockerSshBase.get_path(options)}'
                f' --volume {await GenRosdevHome.get_path(options)}'
                f':{await GenRosdevHome.get_path(options)}'
                f' --volume {await GenWorkspace.get_path(options)}'
                f':{await GenWorkspace.get_path(options)}'
                f'{" --volume /tmp/.X11-unix" if options.docker_container_gui else ""}'
                f' {await GenDockerImage.get_tag(options)}'
                f' /sbin/init'
            ),
            options=options,
        )
        GenDockerContainer.get_id.memoize.reset()
        GenDockerContainer.get_inspect.memoize.reset()
        GenDockerContainer.get_running.memoize.reset()
        log.info(f'Created docker container {await GenDockerContainer.get_name(options)}.')
