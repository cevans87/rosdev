import asyncio
from dataclasses import dataclass, field, replace
import docker
from docker.models.containers import Container
from frozendict import frozendict
from logging import getLogger
import os
from pathlib import Path
from typing import Optional, Tuple, Type

from rosdev.gen.base import GenBase
from rosdev.gen.docker.gdbinit import GenDockerGdbinit
from rosdev.gen.docker.image import GenDockerImage
from rosdev.gen.docker.ssh.port import GenDockerSshPort
from rosdev.gen.rosdev import GenRosdev
from rosdev.gen.ros.install import GenRosInstall
from rosdev.gen.ros.src import GenRosSrc
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerContainer(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenBase,
        GenDockerGdbinit,
        GenDockerImage,
        GenDockerSshPort,
        GenRosdev,
        GenRosInstall,
        GenRosSrc,
    ))

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        docker_container_name = options.resolve_str(options.docker_container_name)

        docker_container_environment = dict(options.docker_container_environment)
        docker_container_environment['ROSDEV_DOCKER_CONTAINER_NAME'] = docker_container_name
        if options.enable_gui:
            docker_container_environment['DISPLAY'] = os.environ['DISPLAY']
        if options.enable_ccache:
            docker_container_environment['CC'] = '/usr/lib/ccache/gcc'
            docker_container_environment['CXX'] = '/usr/lib/ccache/g++'
        docker_container_environment = frozendict(docker_container_environment)

        docker_container_volumes = dict(options.docker_container_volumes)
        docker_container_volumes[options.ros_install_universal_path] = (
            options.ros_install_container_path
        )
        # FIXME move to gen.docker.container.ssh.start
        docker_container_volumes[Path(options.home_universal_path, '.ssh')] = (
            Path(options.home_container_path, '.ssh')
        )
        docker_container_volumes[options.ros_src_universal_path] = options.ros_src_container_path
        docker_container_volumes[options.base_workspace_path] = options.base_container_path
        if options.enable_gui:
            docker_container_volumes['/tmp/.X11-unix'] = '/tmp/.X11-unix'
        docker_container_volumes = frozendict(docker_container_volumes)

        return replace(
            options,
            docker_container_environment=docker_container_environment,
            docker_container_name=docker_container_name,
            docker_container_volumes=docker_container_volumes,
        )

    @classmethod
    async def main(cls, options: Options) -> None:
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
                # TODO also remove if container is based on an old image or fails to restart
                if options.replace_docker_container:
                    log.info(f'Replacing existing docker container')
                    container.remove(force=True)
                    container = None
                elif container.status != 'running':
                    # TODO ensure it starts
                    container.start()
                else:
                    log.info('Docker container is already running')

            if container is None:
                # TODO isolate docker network so multiple workspaces don't interfere with eachother.
                client.containers.run(
                    command=options.docker_container_command,
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
                    working_dir=str(options.base_container_path),
                )

            log.info(f'Started docker container {options.docker_container_name}')

        await asyncio.get_event_loop().run_in_executor(None, main_internal)
