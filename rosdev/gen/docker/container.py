from asyncio import gather
import asyncio
from atools import memoize
from dataclasses import dataclass, field, replace
import docker
from docker.models.containers import Container
from frozendict import frozendict
import json
from logging import getLogger
import os
from pathlib import Path
from typing import Dict, Mapping, Optional, Tuple, Type

from rosdev.gen.core import GenCore
from rosdev.gen.docker.base import GenDockerBase
from rosdev.gen.docker.image import GenDockerImage
from rosdev.gen.host import GenHost
from rosdev.gen.install import GenInstall
from rosdev.gen.src import GenSrc
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerContainer(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenCore,
        GenDockerBase,
        GenDockerImage,
        GenHost,
        GenInstall,
        GenSrc,
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
                f'docker exec'
                f' -e {options.docker_entrypoint_sh_log_level_env_name}={log.getEffectiveLevel()}'
                f' {options.docker_environment_flags}'
                f' {options.docker_container_name}'
                f' {options.docker_entrypoint_sh_container_path}'
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
                f'docker exec'
                f' {options.docker_environment_flags}'
                f' {options.docker_container_name}'
                f' {options.docker_entrypoint_sh_container_path}'
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
                f'docker exec'
                f' {options.docker_environment_flags}'
                f' {options.docker_container_name}'
                f' {options.docker_entrypoint_sh_container_path}'
                f' {command}'
            ),
            options=options,
            err_ok=err_ok,
        )
    
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

        # TODO py38 debug print
        log.debug(f'container_environment: {environment}')

        missing_environment_keys = required_environment_keys - environment.keys()
        if missing_environment_keys:
            log.warning(f'{missing_environment_keys = }')

        return frozendict(environment)

    @classmethod
    async def get_image_id(cls, options: Options) -> str:
        lines = await GenHost.execute_and_get_lines(
            command=f'docker container inspect {options.docker_container_name}', options=options
        )
        return json.loads('\n'.join(lines))[0]['Image']

    @classmethod
    @memoize
    async def get_ssh_port(cls, options: Options) -> int:
        docker_container_ssh_port = int(
            (
                await GenHost.execute_and_get_line(
                    command=f'docker port {options.docker_container_name} 22',
                    options=options,
                )
            ).rsplit(':', 1)[-1]
        )

        log.debug(f'{docker_container_ssh_port = }')

        return docker_container_ssh_port

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        docker_container_environment = dict(options.docker_container_environment)
        if options.docker_container_gui:
            docker_container_environment['DISPLAY'] = os.environ['DISPLAY']
        if options.docker_container_ccache:
            docker_container_environment['CC'] = '/usr/lib/ccache/gcc'
            docker_container_environment['CXX'] = '/usr/lib/ccache/g++'
        docker_container_environment = frozendict(docker_container_environment)

        docker_container_ports = dict(options.docker_container_ports)
        docker_container_ports[22] = None  # let docker assign random port
        docker_container_ports = frozendict(docker_container_ports)

        docker_container_volumes = dict(options.docker_container_volumes)
        docker_container_volumes[options.docker_container_ssh_path] = (
            options.docker_container_ssh_path
        )
        if options.docker_container_gui:
            docker_container_volumes['/tmp/.X11-unix'] = '/tmp/.X11-unix'
        docker_container_volumes = frozendict(docker_container_volumes)

        return replace(
            options,
            docker_container_environment=docker_container_environment,
            docker_container_ports=docker_container_ports,
            docker_container_volumes=docker_container_volumes,
        )
    
    @classmethod
    async def validate_options(cls, options: Options) -> None:
        log.debug(f'{options.docker_container_ccache = }')
        log.debug(f'{options.docker_container_environment = }')
        log.debug(f'{options.docker_container_gui = }')
        log.debug(f'{options.docker_container_ports = }')
        log.debug(f'{options.docker_container_ssh_path = }')
        log.debug(f'{options.docker_container_volumes = }')

    @classmethod
    async def main(cls, options: Options) -> None:
        image_id, install_id, src_id = await gather(*[
            GenDockerImage.get_id(options),
            GenInstall.get_id(options),
            GenSrc.get_id(options),
        ])

        def main_internal() -> None:
            log.info(f'Starting docker container {options.docker_container_name}')
            client = docker.client.from_env()

            # Make sure only one container with our name exists.
            # noinspection PyUnresolvedReferences
            try:
                container: Optional[Container] = client.containers.get(
                    options.docker_container_name
                )
            except docker.errors.NotFound:
                container = None
            else:
                # TODO also remove if container fails to restart
                if (
                        options.docker_container_replace or
                        (container.image.id != image_id) or
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
                    environment=dict(options.docker_container_environment),
                    hostname=options.docker_container_name,
                    image=options.docker_image_tag,
                    ipc_mode='host',
                    name=options.docker_container_name,
                    ports=dict(options.docker_container_ports),
                    privileged=True,  # for X11
                    security_opt=['seccomp=unconfined'],  # for GDB
                    stdin_open=True,
                    tty=True,
                    volumes={
                        str(host_path): {'bind': str(container_path)}
                        for host_path, container_path
                        in options.docker_container_volumes.items()
                    },
                    working_dir=str(Path.cwd()),
                )

            log.info(f'Started docker container {options.docker_container_name}')

        await asyncio.get_event_loop().run_in_executor(None, main_internal)
