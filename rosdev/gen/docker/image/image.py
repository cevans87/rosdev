import asyncio
from atools import memoize
from dataclasses import dataclass, field
import docker
from itertools import product
from logging import getLogger
from io import BytesIO
import os
from textwrap import dedent
from typing import List


log = getLogger(__name__)


@dataclass(frozen=True)
class TempDirContext:
    name: str


@dataclass(frozen=True)
class TempDir:
    cwd: str


@dataclass(frozen=True)
class Dockerfile:
    arch: str
    release: str

    cwd: str = field(init=False, default_factory=os.getcwd)

    @property
    def base_tag(self) -> str:
        return f'{self.arch}/ros:{self.release}-ros-core'

    @property
    def tag(self) -> str:
        return f'{self.base_tag}-dev'

    @property
    def special_commands(self):
        if self.arch == 'amd64':
            return ''

        arch = {
            'arm32v7': 'arm',
            'arm64v8': 'aarch64'
        }[self.arch]

        #return f'COPY /usr/bin/qemu-{arch}-static /usr/bin'
        return f'VOLUME /usr/bin/qemu-{arch}-static'

    @property
    def contents(self) -> str:
        return dedent(f'''
            FROM {self.base_tag}

            {self.special_commands}

            RUN apt-get update
            RUN apt-get install -y \
                build-essential \
                gdbserver \
                openssh-server
            RUN apt-get clean

            RUN python3 -m pip install -U \
                colcon-core \
                colcon-common-extensions \
                pytest \
                pytest-cov

            CMD ln /opt/ros/crystal ${self.cwd}/ros
            CMD source {self.cwd}/ros/setup.bash
        ''')

    @property
    def as_file(self):
        return BytesIO(self.contents.encode())

    @memoize
    async def build(self) -> None:

        def _build() -> None:
            client = docker.client.from_env()
            client.images.build(
                fileobj=self.as_file,
                rm=True,
                tag=self.tag,
                pull=True,
            )

        log.info(f'building "{self.tag}" from "{self.base_tag}"')
        try:
            await asyncio.get_event_loop().run_in_executor(None, _build)
        except docker.errors.BuildError as e:
            log.error(f'while building "{self.tag}", got "{e}"')
        else:
            log.info(f'finished "{self.tag}" from "{self.base_tag}"')


@memoize
async def gen_docker_image(**kwargs) -> None:
    await Dockerfile(**kwargs).build()


async def gen_docker_images(arch: List[str], release: List[str], **kwargs) -> None:
    await asyncio.gather(
        *[gen_docker_image(arch=arch, release=release, **kwargs)
          for arch, release in product(arch, release)])
