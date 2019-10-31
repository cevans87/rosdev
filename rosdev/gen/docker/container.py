import asyncio
from dataclasses import dataclass, field, replace
import docker
from docker.models.containers import Container
from frozendict import frozendict
import json
from logging import getLogger
import os
from pathlib import Path
from typing import Optional, Tuple, Type

from rosdev.gen.core import GenCore
from rosdev.gen.docker.base import GenDockerBase
from rosdev.gen.docker.image import GenDockerImage
from rosdev.gen.docker.ssh.base import GenDockerSshBase
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
        GenDockerSshBase,
        GenInstall,
        GenSrc,
    ))

    @classmethod
    async def get_image_id(cls, options: Options) -> str:
        lines = await cls.execute_host_and_get_lines(
            command=f'docker container inspect {options.docker_container_name}'
        )
        return json.loads('\n'.join(lines))[0]['Image']

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        docker_container_environment = dict(options.docker_container_environment)
        if options.enable_gui:
            docker_container_environment['DISPLAY'] = os.environ['DISPLAY']
        if options.enable_ccache:
            docker_container_environment['CC'] = '/usr/lib/ccache/gcc'
            docker_container_environment['CXX'] = '/usr/lib/ccache/g++'
        docker_container_environment = frozendict(docker_container_environment)

        docker_container_ports = dict(options.docker_container_ports)
        docker_container_ports[22] = options.docker_ssh_port
        docker_container_ports = frozendict(docker_container_ports)

        docker_container_volumes = dict(options.docker_container_volumes)
        # FIXME move to gen.docker.container.ssh.config
        docker_container_volumes[Path(options.home_path, '.ssh')] = (
            Path(options.home_path, '.ssh')
        )
        if options.enable_gui:
            docker_container_volumes['/tmp/.X11-unix'] = '/tmp/.X11-unix'
        docker_container_volumes = frozendict(docker_container_volumes)

        return replace(
            options,
            docker_container_environment=docker_container_environment,
            docker_container_ports=docker_container_ports,
            docker_container_volumes=docker_container_volumes,
        )

    @classmethod
    async def main(cls, options: Options) -> None:
        image_id = await GenDockerImage.get_id(options)

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
                if options.replace_docker_container:
                    log.info(f'Replacing existing docker container.')
                    container.remove(force=True)
                    container = None
                elif container.image.id != image_id:
                    log.info(f'Replacing existing out-of-date docker container.')
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
