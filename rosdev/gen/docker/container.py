from asyncio import gather
import asyncio
from atools import memoize
from dataclasses import dataclass
import docker
from docker.models.containers import Container
from frozendict import frozendict
import json
from logging import getLogger
from pathlib import Path
from typing import Dict, Mapping, Optional, Tuple

from rosdev.gen.docker.entrypoint_sh import GenDockerEntrypointSh
from rosdev.gen.docker.image import GenDockerImage
from rosdev.gen.docker.ssh_base import GenDockerSshBase
from rosdev.gen.home import GenHome
from rosdev.gen.host import GenHost
from rosdev.gen.install import GenInstall
from rosdev.gen.install_base import GenInstallBase
from rosdev.gen.src import GenSrc
from rosdev.gen.src_base import GenSrcBase
from rosdev.gen.workspace import GenWorkspace
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerContainer(Handler):

    @classmethod
    @memoize
    async def get_environment(cls, options: Options) -> Mapping[str, str]:
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
        for line in await cls.execute_and_get_lines(command=f'env', options=options):
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
        
        log.debug(f'{cls.__name__} {environment = }')
        
        return environment

    @classmethod
    @memoize
    async def get_image_id(cls, options: Options) -> str:
        lines = await GenHost.execute_and_get_lines(
            command=f'docker container inspect {await cls.get_name(options)}', options=options
        )
        image_id = json.loads('\n'.join(lines))[0]['Image']

        log.debug(f'{cls.__name__} {image_id = }')
        
        return image_id

    @classmethod
    @memoize
    async def get_name(cls, options: Options) -> str:
        name = (
            f'rosdev_{options.release}_{options.architecture}_'
            f'{await GenWorkspace.get_hash(options)}'
        )
        
        log.debug(f'{cls.__name__} {name = }')
        
        return name

    @classmethod
    @memoize
    async def get_ports(cls, options: Options) -> Mapping[int, Optional[int]]:
        # FIXME if someone specifies additional ports after the container is already running,
        #  they will not be mapped. Rebuild the container with the new ports added.

        ports: Dict[int, Optional[int]] = dict(options.docker_container_ports)
        ports[22] = None  # let docker assign random port
        ports: Mapping[int, Optional[int]] = frozendict(ports)
        
        log.debug(f'{cls.__name__} {ports = }')
        
        return ports

    @classmethod
    @memoize
    async def get_ssh_path(cls, options: Options) -> Path:
        ssh_path = await GenHome.get_path(options) / '.ssh'
        
        log.debug(f'{cls.__name__} {ssh_path = }')
        
        return ssh_path

    @classmethod
    @memoize
    async def get_volumes(cls, options: Options) -> Mapping[Path, Path]:
        volumes = dict(options.docker_container_volumes)
        for path in await gather(*[
            GenDockerSshBase.get_path(options),
            GenWorkspace.get_path(options),
            GenInstallBase.get_path(options),
            GenSrcBase.get_path(options),
        ]):
            volumes[path] = path

        if options.docker_container_gui:
            volumes['/tmp/.X11-unix'] = '/tmp/.X11-unix'
        volumes = frozendict(volumes)

        log.debug(f'{cls.__name__} {volumes = }')

        return volumes

    @classmethod
    @memoize
    async def execute(
            cls,
            *,
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
                f' {await cls.get_name(options)}'
                f' {await GenDockerEntrypointSh.get_container_path(options)}'
                f' {command}'
            ),
            err_ok=err_ok,
            options=options,
        )

    @classmethod
    @memoize
    async def execute_and_get_lines(
            cls,
            *,
            command: str,
            options: Options,
            err_ok=False,
    ) -> Tuple[str]:
        return await GenHost.execute_and_get_lines(
            command=(
                f'docker exec'
                f' {await GenDockerEntrypointSh.get_environment_flags(options)}'
                f' {await cls.get_name(options)}'
                f' {await GenDockerEntrypointSh.get_container_path(options)}'
                f' {command}'
            ),
            err_ok=err_ok,
            options=options,
        )

    @classmethod
    @memoize
    async def execute_and_get_line(
            cls,
            *,
            command: str,
            options: Options,
            err_ok=False,
    ) -> str:
        return await GenHost.execute_and_get_line(
            command=(
                f'docker exec'
                f' {await GenDockerEntrypointSh.get_environment_flags(options)}'
                f' {await cls.get_name(options)}'
                f' {await GenDockerEntrypointSh.get_container_path(options)}'
                f' {command}'
            ),
            options=options,
            err_ok=err_ok,
        )
    
    @classmethod
    async def main(cls, options: Options) -> None:
        (
            docker_entrypoint_sh_environment,
            docker_image_id,
            docker_image_tag,
            docker_container_name,
            docker_container_ports,
            docker_container_volumes,
            install_id,
            src_id,
        ) = await gather(*[
            GenDockerEntrypointSh.get_environment(options),
            GenDockerImage.get_id(options),
            GenDockerImage.get_tag(options),
            cls.get_name(options),
            cls.get_ports(options),
            cls.get_volumes(options),
            GenInstall.get_id(options),
            GenSrc.get_id(options),
        ])

        def main_internal() -> None:
            client = docker.client.from_env()

            # Make sure only one container with our name exists.
            # noinspection PyUnresolvedReferences
            try:
                container: Optional[Container] = client.containers.get(
                    docker_container_name
                )
            except docker.errors.NotFound:
                container = None
            else:
                # TODO also remove if container fails to restart
                if (
                        options.docker_container_replace or
                        (container.image.id != docker_image_id) or
                        (container.image.id != install_id) or
                        (container.image.id != src_id)
                ):
                    log.info(f'Replacing existing docker container.')
                    container.remove(force=True)
                    container = None
                elif container.status != 'running':
                    # TODO ensure it starts
                    container.start()
                else:
                    log.info('Docker container is already running.')

            if container is None:
                # TODO isolate docker network so multiple workspaces don't interfere with eachother.
                client.containers.run(
                    command='/sbin/init',
                    detach=True,
                    environment=dict(docker_entrypoint_sh_environment),
                    hostname=docker_container_name,
                    image=docker_image_tag,
                    ipc_mode='host',
                    name=docker_container_name,
                    ports=dict(docker_container_ports),
                    privileged=True,  # for X11
                    security_opt=['seccomp=unconfined'],  # for GDB
                    stdin_open=True,
                    tty=True,
                    volumes={
                        str(host_path): {'bind': str(container_path)}
                        for host_path, container_path
                        in docker_container_volumes.items()
                    },
                    working_dir=str(Path.cwd()),
                )

        log.info(f'Starting docker container {await cls.get_name(options)}')
        await asyncio.get_event_loop().run_in_executor(None, main_internal)
        log.info(f'Started docker container {await cls.get_name(options)}')
