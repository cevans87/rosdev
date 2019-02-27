from atools import memoize
import docker
from logging import getLogger
import os
from pathlib import Path
from typing import FrozenSet, Optional

from rosdev.gen.docker.images import image
from rosdev.util.subprocess import exec


log = getLogger(__package__)


@memoize
async def container(
        architecture: str,
        build_num: Optional[int],
        command: str,
        fast: bool,
        interactive: bool,
        ports: FrozenSet[int],
        release: str,
) -> None:
    dockerfile = await image(architecture=architecture, fast=fast, release=release)

    cwd = os.getcwd()
    home = str(Path.home())
    volumes = {home: {'bind': home}}
    if not cwd.startswith(home):
        volumes[cwd] = {'bind': cwd}

    environment = {k: v for k, v in os.environ.items() if 'AWS' in k}
    environment['ROSDEV_DIR'] = os.getcwd()
    environment['ROSDEV_INSTALL_DIR'] = \
        f'{os.getcwd()}/.rosdev/{architecture}/{build_num or release}'

    client = docker.client.from_env()
    container = client.containers.create(
        auto_remove=True,
        command=command,
        detach=True,
        environment=environment,
        image=dockerfile.tag,
        ports={port: port for port in ports},
        security_opt=['seccomp=unconfined'],
        stdin_open=True,
        tty=True,
        volumes=volumes,
        working_dir=cwd,
    )

    log.debug(f'attaching to container "{container.name}"')
    if interactive:
        os.execlpe('docker', *f'docker start -ai {container.name}'.split(), os.environ)
    else:
        await exec(f'docker start -a {container.name}')
