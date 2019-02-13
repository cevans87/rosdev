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
class Dockerfile:
    architecture: str
    release: str

    cwd: str = field(init=False, default_factory=os.getcwd)

    @property
    def base_tag(self) -> str:
        return f'{self.architecture}/ros:{self.release}-ros-core'

    @property
    def tag(self) -> str:
        return f'{self.base_tag}-dev'

    @property
    def special_commands(self):
        if self.architecture == 'amd64':
            return ''

        architecture = {
            'arm32v7': 'arm',
            'arm64v8': 'aarch64'
        }[self.architecture]

        return f'VOLUME /usr/bin/qemu-{architecture}-static'

    @property
    def contents(self) -> str:
        return dedent(f'''
            FROM {self.base_tag}

            {self.special_commands}

            RUN apt-get update
            RUN apt-get install -y \
                build-essential \
                gdbserver \
                openssh-server \
                python3-pip \
                sudo
            RUN apt-get clean

            RUN python3 -m pip install -U \
                colcon-core \
                colcon-common-extensions \
                pytest \
                pytest-cov \
                vcstool

            RUN groupadd -r -g {os.getgid()} {os.getlogin()}
            RUN useradd {os.getlogin()} -r -u {os.getuid()} -g {os.getgid()} -G sudo 1>/dev/null
            RUN echo "{os.getlogin()} ALL=(ALL:ALL) NOPASSWD: ALL" >> /etc/sudoers

            USER {os.getlogin()}
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
async def gen_docker_image(architecture: str, release: str) -> Dockerfile:
    dockerfile = Dockerfile(architecture, release)
    await dockerfile.build()

    return dockerfile


async def gen_docker_images(architecture: List[str], release: List[str]) -> None:
    await asyncio.gather(
        *[gen_docker_image(architecture, release) for architecture, release in product(architecture, release)])
