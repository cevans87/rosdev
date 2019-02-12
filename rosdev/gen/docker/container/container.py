import asyncio
from atools import memoize
import docker
from itertools import product
from logging import getLogger
import os
from typing import List

from ..image.image import gen_docker_image


log = getLogger(__name__)


@memoize
async def gen_docker_container(arch: str, release: str, attach: bool) -> None:
    dockerfile = await gen_docker_image(arch, release)

    cwd = os.getcwd()
    client = docker.client.from_env()
    container = client.containers.create(
        image=dockerfile.tag,
        command='/bin/bash',
        tty=True,
        detach=True,
        stdin_open=True,
        auto_remove=True,
        volumes={cwd: {'bind': cwd}},
        environment={k: v for k, v in os.environ.items() if 'AWS' in k},
    )

    if attach:
        log.info(f'attaching to "{container.name}"')
        os.execlpe('docker', *f'docker start -ai {container.name}'.split(), os.environ)


async def gen_docker_containers(arch: List[str], release: List[str], attach: bool) -> None:
    await asyncio.gather(
        *[gen_docker_container(arch, release, attach) for arch, release in product(arch, release)])
