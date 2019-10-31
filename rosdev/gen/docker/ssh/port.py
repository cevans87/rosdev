from ast import literal_eval
from dataclasses import dataclass, field, replace
from logging import getLogger
import socket
from typing import Tuple, Type

from rosdev.gen.base import GenBase
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerSshPort(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenBase,
    ))

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:

        docker_ssh_port = options.docker_ssh_port
        if docker_ssh_port is None:
            try:
                docker_ssh_port = int(
                    literal_eval(options.docker_ssh_port_path.read_text())
                )
            except (FileNotFoundError, PermissionError, ValueError):
                tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                tcp.bind(('', 0))
                _, docker_ssh_port = tcp.getsockname()
                tcp.close()

        return replace(options, docker_ssh_port=docker_ssh_port)

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        log.debug(f'{options.docker_ssh_port = }')
        log.debug(f'{options.docker_ssh_port_path = }')

    @classmethod
    async def main(cls, options: Options) -> None:
        options.write_text(
            path=options.docker_ssh_port_path,
            text=f'{options.docker_ssh_port}'
        )
