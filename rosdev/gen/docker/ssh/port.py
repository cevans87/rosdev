from dataclasses import dataclass, field, replace
from frozendict import frozendict
from logging import getLogger
import socket
from typing import Tuple, Type

from rosdev.gen.rosdev import GenRosdev
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerSshPort(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenRosdev,
    ))

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:

        docker_ssh_workspace_port_path = options.resolve_path(
            options.docker_ssh_workspace_port_path
        )

        docker_ssh_workspace_port = options.docker_ssh_workspace_port
        if docker_ssh_workspace_port is None:
            try:
                with open(str(docker_ssh_workspace_port_path), 'r') as f_in:
                    docker_ssh_workspace_port = int(f_in.read())
            except (FileNotFoundError, PermissionError, ValueError):
                tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                tcp.bind(('', 0))
                _, docker_ssh_workspace_port = tcp.getsockname()
                tcp.close()

        docker_container_ports = dict(options.docker_container_ports)
        docker_container_ports[22] = docker_ssh_workspace_port
        docker_container_ports = frozendict(docker_container_ports)

        return replace(
            options,
            docker_container_ports=docker_container_ports,
            docker_ssh_workspace_port=docker_ssh_workspace_port,
            docker_ssh_workspace_port_path=docker_ssh_workspace_port_path,
        )

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        # TODO py38 debug print
        log.debug(f'docker_ssh_workspace_port: {options.docker_ssh_workspace_port}')
        log.debug(f'docker_ssh_workspace_port_path: {options.docker_ssh_workspace_port_path}')

    @classmethod
    async def main(cls, options: Options) -> None:
        with open(str(options.docker_ssh_workspace_port_path), 'w') as f_out:
            f_out.write(f'{options.docker_ssh_workspace_port}')
