import asyncio
from atools import memoize
from dataclasses import dataclass
import docker
from itertools import product
from logging import getLogger
import os
import pathlib
import platform
from tempfile import TemporaryDirectory
from textwrap import dedent
from typing import FrozenSet

from rosdev.util.lookup import get_machine


log = getLogger(__package__)


@memoize
@dataclass(frozen=True)
class Image:
    architecture: str
    fast: bool
    release: str

    @memoize
    async def __call__(self) -> None:

        def ___call___internal() -> None:
            with TemporaryDirectory() as tempdir_path:
                with open(f'{tempdir_path}/Dockerfile', 'w') as dockerfile_f_out:
                    dockerfile_f_out.write(self.contents)
                with open(f'{tempdir_path}/rosdev_entrypoint.sh', 'w') as entrypoint_f_out:
                    entrypoint_f_out.write(self.entrypoint.contents)
                with open(f'{pathlib.Path.home()}/.ssh/id_rsa.pub', 'r') as id_rsa_f_in, \
                        open(f'{tempdir_path}/id_rsa.pub', 'w') as id_rsa_f_out:
                    id_rsa_f_out.write(id_rsa_f_in.read())

                client = docker.client.from_env()
                client.images.build(
                    path=tempdir_path,
                    pull=not self.fast,
                    rm=True,
                    tag=self.tag,
                )

        log.info(f'building "{self.tag}" from "{self.base_tag}"')
        try:
            await asyncio.get_event_loop().run_in_executor(None, ___call___internal)
        except docker.errors.BuildError as e:
            log.error(f'while building "{self.tag}", got "{e}"')
        else:
            log.info(f'finished "{self.tag}" from "{self.base_tag}"')

    @property
    @memoize
    def cwd(self) -> str:
        return os.getcwd()

    @property
    def base_tag(self) -> str:
        if self.release == 'latest':
            return f'osrf/ros2:nightly'
        else:
            return f'{self.architecture}/ros:{self.release}-ros-core'

    @property
    def tag(self) -> str:
        return f'{self.base_tag}-dev'

    @property
    def machine(self) -> str:
        return get_machine(self.architecture)

    @property
    def qemu_path(self) -> str:
        return f'/usr/bin/qemu-{self.machine}-static'

    @property
    def contents(self) -> str:
        return dedent(fr'''
            FROM {self.base_tag}

            # qemu static binaries
            {f'VOLUME {self.qemu_path}' if platform.machine() != self.machine else '# not needed'}

            # make ssh easier
            #VOLUME /etc/ssh

            RUN apt-get update
            RUN apt-get install -y \
                build-essential \
                coreutils \
                curl \
                gdb \
                gdbserver \
                openssh-server \
                libcurl4-openssl-dev \
                python3-pip \
                sudo
            RUN apt-get clean

            RUN python3 -m pip install -U \
                colcon-core \
                colcon-common-extensions \
                pytest \
                pytest-cov \
                vcstool

            RUN rm /ros_entrypoint.sh
            COPY rosdev_entrypoint.sh /
            RUN chmod +x /rosdev_entrypoint.sh

            RUN groupadd -r -g {os.getgid()} {os.getlogin()}
            RUN useradd {os.getlogin()} -l -r -u {os.getuid()} -g {os.getgid()} -G sudo 1>/dev/null
            RUN echo "{os.getlogin()} ALL=(ALL:ALL) NOPASSWD: ALL" >> /etc/sudoers

            USER {os.getlogin()}

            #RUN mkdir -p {pathlib.Path.home()}/.ssh
            #RUN touch {pathlib.Path.home()}/.ssh/authorized_keys
            #RUN cat /id_rsa.pub >> {pathlib.Path.home()}/.ssh/authorized_keys
            #RUN chmod 600 {pathlib.Path.home()}/.ssh/authorized_keys

            ENTRYPOINT ["/rosdev_entrypoint.sh"]
            CMD bash
        ''').lstrip()


@memoize
@dataclass(frozen=True)
class Images:
    architectures: FrozenSet[str]
    fast: bool
    releases: FrozenSet[str]

    @memoize
    async def __call__(self) -> None:
        await asyncio.gather(
            *[Image(architecture=architecture, fast=self.fast, release=release)()
              for architecture, release in product(self.architectures, self.releases)])
