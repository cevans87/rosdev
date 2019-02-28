from atools import memoize
from dataclasses import dataclass
import docker
from logging import getLogger
import os
from pathlib import Path
from typing import FrozenSet, Optional

from rosdev.gen.docker.images import Image
from rosdev.util.handler import Handler
from rosdev.util.subprocess import exec


log = getLogger(__package__)


@memoize
@dataclass(frozen=True)
class Container(Handler):
    architecture: str
    build_num: Optional[int]
    command: str
    fast: bool
    interactive: bool
    ports: FrozenSet[int]
    release: str

    @property
    @memoize
    def image(self) -> Image:
        return Image(architecture=self.architecture, fast=self.fast, release=self.release)

    @memoize
    async def exit_code(self) -> int:
        await self.image

        cwd = os.getcwd()
        home = str(Path.home())
        volumes = {home: {'bind': home}}
        if not cwd.startswith(home):
            volumes[cwd] = {'bind': cwd}

        environment = {k: v for k, v in os.environ.items() if 'AWS' in k}
        environment['ROSDEV_DIR'] = os.getcwd()
        environment['ROSDEV_INSTALL_DIR'] = \
            f'{os.getcwd()}/.rosdev/{self.architecture}/{self.build_num or self.release}'

        client = docker.client.from_env()
        container = client.containers.create(
            auto_remove=True,
            command=self.command,
            detach=True,
            environment=environment,
            image=self.image.tag,
            ports={port: port for port in self.ports},
            security_opt=['seccomp=unconfined'],
            stdin_open=True,
            tty=True,
            volumes=volumes,
            working_dir=cwd,
        )

        log.debug(f'attaching to container "{container.name}"')
        if self.interactive:
            os.execlpe('docker', *f'docker start -ai {container.name}'.split(), os.environ)

        return await exec(f'docker start -a {container.name}')

    @memoize
    async def _run(self) -> None:
        await self.exit_code()
