from dataclasses import dataclass
import getpass
from logging import getLogger
from typing import Dict, Tuple

from rosdev.gen.docker.container_base import GenDockerContainerBase
from rosdev.gen.docker.gdbinit import GenDockerGdbinit
from rosdev.gen.docker.entrypoint_sh import GenDockerEntrypointSh
from rosdev.gen.docker.image import GenDockerImage
from rosdev.gen.docker.ssh_base import GenDockerSshBase
from rosdev.gen.host import GenHost
from rosdev.gen.install import GenInstall
from rosdev.gen.src import GenSrc
from rosdev.util.atools import memoize, memoize_db
from rosdev.util.frozendict import frozendict, FrozenDict
from rosdev.util.options import Options
from rosdev.util.path import Path
from rosdev.util.uri import Uri


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerContainer(GenDockerContainerBase):

    @staticmethod
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
    async def get_environment(options: Options) -> FrozenDict[str, str]:
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

        environment: FrozenDict[str, str] = frozendict(environment)

        log.debug(f'{__class__.__name__} {environment = }')

        return environment

    @staticmethod
    @memoize
    async def get_ip(options: Options) -> str:
        ip = (await GenDockerContainer._get_inspect_json(options))['NetworkSettings']['IPAddress']

        log.debug(f'{__class__.__name__} {ip = }')

        return ip

    @staticmethod
    @memoize
    async def get_port(options: Options) -> int:
        port = int(
            (
                await GenDockerContainer._get_inspect_json(options)
            )['NetworkSettings']['Ports']['22/tcp'][0]['HostPort']
        )

        log.debug(f'{__class__.__name__} {port = }')

        return port

    @staticmethod
    @memoize
    async def get_uri(options) -> Uri:
        uri = Uri(f'ssh://localhost:{await GenDockerContainer.get_port(options)}')

        log.debug(f'{__class__.__name__} {uri = }')

        return uri

    @staticmethod
    @memoize_db(keygen=lambda options: GenDockerContainerBase.get_id(options), size=1)
    async def _main_inner(options: Options) -> None:
        if await GenDockerContainer._get_inspect_json(options):
            await GenHost.execute(
                command=f'docker container rm -f {await GenDockerContainerBase.get_name(options)}',
                options=options,
            )
        GenDockerContainer._get_inspect_json.memoize.reset()

        await GenHost.execute(
            command=(
                f'docker run'
                f' --detach'
                f' --expose 22'
                f' --hostname {await GenDockerContainerBase.get_name(options)}'
                f' --ipc host'
                f' --name {await GenDockerContainerBase.get_name(options)}'
                f' --privileged'
                f' --publish-all'
                f' --security-opt seccomp=unconfined'
                f' --volume {await GenDockerSshBase.get_path(options)}'
                f':{await GenDockerSshBase.get_path(options)}'
                f' --volume {Path.workspace()}:{Path.workspace()}'
                f' --volume {Path.volume()}:{Path.volume()}'
                f'{" --volume /tmp/.X11-unix" if options.docker_container_gui else ""}'
                f' {await GenDockerImage.get_tag(options)}'
                f' /sbin/init'
            ),
            options=options,
        )
        await GenDockerContainer.execute(
            command=(
                f'sudo chown -R {getpass.getuser()}:{getpass.getuser()} {Path.rosdev().resolve()}'
            ),
            options=options,
        )
        await GenDockerContainer.execute(
            command=f'mkdir -p {Path.store()}',
            options=options,
        )
        await GenDockerContainer.execute(
            command=(
                f'ln -s'
                f' {await GenInstall.get_container_path(options)}'
                f' {await GenInstall.get_path(options)}'
            ),
            options=options,
        )
        await GenDockerContainer.execute(
            command=(
                f'ln -s'
                f' {await GenSrc.get_container_path(options)}'
                f' {await GenSrc.get_path(options)}'
            ),
            options=options,
        )
        await GenDockerContainer.execute(
            command=(
                f' ln -s'
                f' {await GenDockerGdbinit.get_path(options)}'
                f' {await GenDockerGdbinit.get_container_path(options)}'
            ),
            options=options,
        )
        log.info(f'Created docker container {await GenDockerContainer.get_name(options)}.')

    @staticmethod
    @memoize
    async def main(options: Options) -> None:

        await GenDockerContainer._main_inner(options)

        if not (await GenDockerContainer._get_inspect_json(options))['State']['Running']:
            GenDockerContainer._get_inspect_json.memoize.reset()

            log.info('Restarting stopped docker container.')
            await GenHost.execute(
                command=f'docker start {await GenDockerContainerBase.get_name(options)}',
                options=options,
            )
