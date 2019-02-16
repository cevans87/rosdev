from atools import memoize
import docker
from logging import getLogger
import os
from pathlib import Path
from typing import FrozenSet

from ..images.images import gen_docker_image


log = getLogger(__name__)


@memoize
async def gen_docker_container(
        architecture: str,
        release: str,
        ports: FrozenSet[int],
        interactive: bool,
        command: str,
) -> None:
    dockerfile = await gen_docker_image(architecture, release)

    cwd = os.getcwd()
    home = str(Path.home())
    volumes = {home: {'bind': home}}
    if not cwd.startswith(home):
        volumes[cwd] = {'bind': cwd}

    client = docker.client.from_env()
    container = client.containers.create(
        image=dockerfile.tag,
        command=command,
        tty=True,
        detach=True,
        stdin_open=True,
        auto_remove=True,
        volumes=volumes,
        working_dir=cwd,
        ports={port: port for port in ports},
        environment={k: v for k, v in os.environ.items() if 'AWS' in k},
    )

    if interactive:
        log.info(f'attaching to "{container.name}"')
        os.execlpe('docker', *f'docker start -ai {container.name}'.split(), os.environ)
