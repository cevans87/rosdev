import asyncio
from atools import memoize
from dataclasses import dataclass, field
import docker
from itertools import product
from logging import getLogger
import os
import pathlib
import platform
from tempfile import TemporaryDirectory
from textwrap import dedent
from typing import FrozenSet


log = getLogger(__package__)


@dataclass(frozen=True)
class Entrypoint:
    architecture: str
    release: str


    @property
    def source_global_setup(self) -> str:
        return ' > /dev/null 2>&1 || '.join([
            'source "$ROSDEV_INSTALL_DIR/setup.bash"',
            'source "/opt/ros/$ROS_DISTRO/setup.bash"',
            ':'
        ])

    @property
    def source_local_setup(self) -> str:
        return ' 2> /dev/null || '.join([
            'source "$ROSDEV_DIR/install/setup.bash"',
            ':'
        ])

    @property
    def contents(self) -> str:
        return dedent(fr'''
            #!/bin/bash
            set -e

            # setup ros2 environment
            {self.source_global_setup}
            {self.source_local_setup}

            exec "$@"
        ''').lstrip()


@dataclass(frozen=True)
class Dockerfile:
    architecture: str
    release: str

    cwd: str = field(init=False, default_factory=os.getcwd)

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
        return {
            'amd64': 'x86_64',
            'arm32v7': 'arm',
            'arm64v8': 'aarch64'
        }[self.architecture]

    @property
    def qemu_path(self) -> str:
        return f'/usr/bin/qemu-{self.machine}-static'

    @property
    def entrypoint(self) -> Entrypoint:
        return Entrypoint(architecture=self.architecture, release=self.release)

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
    async def build(self) -> None:

        def _build() -> None:
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
                    pull=True,
                    rm=True,
                    tag=self.tag,
                )

        log.info(f'building "{self.tag}" from "{self.base_tag}"')
        try:
            await asyncio.get_event_loop().run_in_executor(None, _build)
        except docker.errors.BuildError as e:
            log.error(f'while building "{self.tag}", got "{e}"')
        else:
            log.info(f'finished "{self.tag}" from "{self.base_tag}"')


@memoize
async def image(
        *,
        architecture: str,
        release: str
) -> Dockerfile:
    dockerfile = Dockerfile(architecture=architecture, release=release)
    await dockerfile.build()

    return dockerfile


@memoize
async def images(
        *,
        architectures: FrozenSet[str],
        releases: FrozenSet[str]
) -> None:
    await asyncio.gather(
        *[image(architecture=architecture, release=release)
          for architecture, release in product(architectures, releases)])