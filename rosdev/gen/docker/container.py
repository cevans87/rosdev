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
    clean: bool
    command: str
    fast: bool
    interactive: bool
    ports: FrozenSet[int]
    release: str

    class Exception(Exception):
        pass

    @property
    @memoize
    def image(self) -> Image:
        return Image(architecture=self.architecture, fast=self.fast, release=self.release)

    @memoize
    async def exit_code(self) -> int:
        await self.image

        environment = {k: v for k, v in os.environ.items() if 'AWS' in k}
        if self.clean:
            environment['ROSDEV_CLEAN_ENVIRONMENT'] = '1'
        else:
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
            volumes={
                os.getcwd(): {'bind': os.getcwd()},
                str(Path.home()): {'bind': str(Path.home())},
            },
            working_dir=os.getcwd(),
        )

        log.debug(f'attaching to container "{container.name}"')
        if self.interactive:
            os.execlpe('docker', *f'docker start -ai {container.name}'.split(), os.environ)

        return await exec(f'docker start -a {container.name}')

    @memoize
    async def must_succeed(self) -> None:
        if await self.exit_code() != 0:
            raise self.Exception('Build failed.')

    @memoize
    async def _main(self) -> None:
        await self.exit_code()
