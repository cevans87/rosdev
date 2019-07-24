from dataclasses import dataclass, field, replace
import docker
from docker.models.containers import Container
from frozendict import frozendict
from logging import getLogger
import os
from pathlib import Path
from typing import Optional, Tuple, Type

from rosdev.gen.base import GenBase
from rosdev.gen.docker.image import GenDockerImage
from rosdev.gen.rosdev import GenRosdev
from rosdev.gen.ros.install import GenRosInstall
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerContainer(Handler):
    # FIXME depend on overlay/underlay
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenBase,
        GenDockerImage,
        GenRosdev,
        GenRosInstall,
    ))

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:

        docker_container_name = options.resolve_str(options.docker_container_name)
        #docker_container_name += f'_{hash(str(options.base_workspace_path))}'

        docker_container_environment = dict(options.docker_container_environment)
        docker_container_environment['ROSDEV_DOCKER_CONTAINER_NAME'] = docker_container_name
        if options.source_ros_setup_overlay:
            docker_container_environment['ROSDEV_ROS_SETUP_OVERLAY_CONTAINER_PATH'] = (
                str(options.ros_setup_overlay_container_path)
            )
        if options.source_ros_setup_underlay:
            docker_container_environment['ROSDEV_ROS_SETUP_UNDERLAY_CONTAINER_PATH'] = (
                # FIXME move this resolve_container_path to underlay
                str(options.resolve_container_path(options.ros_setup_underlay_container_path))
            )
        if options.enable_gui:
            docker_container_environment['DISPLAY'] = os.environ['DISPLAY']
        if options.enable_ccache:
            docker_container_environment['CC'] = '/usr/lib/ccache/gcc'
            docker_container_environment['CXX'] = '/usr/lib/ccache/g++'
        docker_container_environment = frozendict(docker_container_environment)

        docker_container_volumes = dict(options.docker_container_volumes)
        #docker_container_volumes[options.rosdev_universal_path] = options.rosdev_universal_path
        docker_container_volumes[options.ros_install_universal_path] = (
            options.ros_install_container_path
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
        client = docker.client.from_env()

        # Make sure only one container with our name exists.
        # noinspection PyUnresolvedReferences
        try:
            container: Optional[Container] = client.containers.get(options.docker_container_name)
        except docker.errors.NotFound:
            container = None
        else:
            # TODO also remove if container is based on an old image
            if options.replace_docker_container:
                log.info(f'Replacing existing docker container "{options.docker_container_name}"')
                container.remove(force=True)
                container = None
            elif container.status != 'running':
                container.start()

        if container is None:
            container = client.containers.run(
                command='/sbin/init',
                detach=True,
                environment={k: v for k, v in options.docker_container_environment.items()},
                image=options.docker_image_tag,
                ipc_mode='host',
                name=options.docker_container_name,
                ports={port: port for port in options.ports},
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

        if options.docker_container_command is not None:
            if options.interactive_docker_container:
                os.execlpe(
                    'docker',
                    *(
                        f'docker exec -it {container.name} '
                        f'/rosdev_entrypoint.sh {options.docker_container_command}'
                    ).split(),
                    os.environ
                )
            else:
                container.exec_run(
                    cmd=options.docker_container_command,
                    detach=True,
                    privileged=True,
                    user=os.getlogin(),
                    workdir=str(options.base_container_path),
                )
