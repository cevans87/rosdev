from atools import memoize
import docker
from logging import getLogger
import os
from pathlib import Path
from typing import FrozenSet

from ..images.images import image


log = getLogger(__name__)


@memoize
async def container(
        architecture: str,
        nightly: bool,
        release: str,
        ports: FrozenSet[int],
        interactive: bool,
        command: str,
) -> None:
    dockerfile = await image(architecture, nightly, release)

    cwd = os.getcwd()
    home = str(Path.home())
    volumes = {home: {'bind': home}}
    if not cwd.startswith(home):
        volumes[cwd] = {'bind': cwd}

    environment = {k: v for k, v in os.environ.items() if 'AWS' in k}
    environment['ROSDEV_DIR'] = os.getcwd()

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
        environment=environment,
        security_opt=['seccomp=unconfined'],
    )

    if interactive:
        log.info(f'attaching to "{container.name}"')
        os.execlpe('docker', *f'docker start -ai {container.name}'.split(), os.environ)
