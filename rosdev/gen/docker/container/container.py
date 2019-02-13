import asyncio
from atools import memoize
import docker
from itertools import product
from logging import getLogger
import os
from pathlib import Path
from typing import FrozenSet, List

from ..image.image import gen_docker_image


log = getLogger(__name__)


@memoize
async def gen_docker_container(
        architecture: str,
        release: str,
        interactive: bool,
        ports: FrozenSet[int],
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
        command='/bin/bash',
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


async def gen_docker_containers(
        architecture: List[str],
        release: List[str],
        interactive: bool,
        ports: List[int],
) -> None:
    await asyncio.gather(
        *[gen_docker_container(architecture, release, interactive, frozenset(ports))
          for architecture, release in product(architecture, release)])
