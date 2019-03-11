from __future__ import annotations
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

from rosdev.util.handler import Handler
from rosdev.util.lookup import get_machine


log = getLogger(__package__)


@memoize
@dataclass(frozen=True)
class Image(Handler):
    architecture: str
    fast: bool
    release: str

    class Exception(Exception):
        pass

    @memoize
    async def _run(self) -> None:
        def _run_internal() -> None:
            with TemporaryDirectory() as tempdir_path:
                with open(f'{tempdir_path}/Dockerfile', 'w') as dockerfile_f_out:
                    dockerfile_f_out.write(self.dockerfile_contents)
                with open(f'{tempdir_path}/rosdev_entrypoint.sh', 'w') as entrypoint_f_out:
                    entrypoint_f_out.write(self.rosdev_entrypoint_sh_contents)
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
            await asyncio.get_event_loop().run_in_executor(None, _run_internal)
        except docker.errors.BuildError as e:
            raise self.Exception(f'While building "{self.tag}", got "{e}"') from e
        else:
            log.info(f'Finished "{self.tag}" from "{self.base_tag}"')

    @property
    @memoize
    def cwd(self) -> str:
        return os.getcwd()

    @property
    def base_tag(self) -> str:
        if self.release != 'latest':
            return f'{self.architecture}/ros:{self.release}-ros-core'
        elif self.architecture != 'amd64':
            return f'{self.architecture}/ros:crystal-ros-core'
        else:
            return f'osrf/ros2:nightly'

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
    def rosdev_entrypoint_sh_contents(self) -> str:
        return dedent(fr'''
            #!/bin/bash
            set -e
            # setup ros2 environment
            if [ -z ${{ROSDEV_CLEAN_ENVIRONMENT+x}} ]; then \
                source "$ROSDEV_DIR/install/setup.bash" > /dev/null 2>&1 || \
                source "$ROSDEV_INSTALL_DIR/setup.bash" > /dev/null 2>&1 || \
                source "/opt/ros/$ROS_DISTRO/setup.bash" > /dev/null 2>&1 || \
                :; \
            fi

            exec "$@"
        ''').lstrip()

    @property
    def dockerfile_contents(self) -> str:
        # FIXME see how hard it is to host this image on Dockerhub. It takes a while to build.
        return dedent(fr'''
            FROM {self.base_tag}

            # qemu static binaries
            {f'VOLUME {self.qemu_path}' if platform.machine() != self.machine
                and platform.system() != 'Darwin' else '# not needed'}

            # make ssh easier
            #VOLUME /etc/ssh

            # XXX arm32v8 and arm64v8 return error code 100 if we only apt-get update once.
            # see https://github.com/rocker-org/shiny/issues/19#issuecomment-308357402
            RUN apt-get update && apt-get update && apt-get install -y \
                build-essential \
                coreutils \
                curl \
                gdb \
                gdbserver \
                openssh-server \
                python3-pip \
                sudo \
                && apt-get clean

                # These are for building all of ros2
                #build-essential \
                #cmake \
                #git \
                #python3-colcon-common-extensions \
                #python3-pip \
                #python-rosdep \
                #python3-vcstool \
                #wget \
                #libasio-dev \
                #libtinyxml2-dev \

            RUN python3 -m pip install -U \
                argcomplete \
                colcon-core \
                colcon-common-extensions \
                pytest \
                pytest-cov \
                vcstool

                # These are for building all of ros2
                #flake8 \
                #flake8-blind-except \
                #flake8-builtins \
                #flake8-class-newline \
                #flake8-comprehensions \
                #flake8-deprecated \
                #flake8-docstrings \
                #flake8-import-order \
                #flake8-quotes \
                #git+https://github.com/lark-parser/lark.git@0.7d \
                #pytest-repeat \
                #pytest-rerunfailures \
                #pytest \
                #pytest-cov \
                #pytest-runner \
                #setuptools

            RUN rm /ros_entrypoint.sh
            COPY rosdev_entrypoint.sh /
            RUN chmod +x /rosdev_entrypoint.sh

            RUN groupadd -r -g {os.getgid()} {os.getlogin()}
            RUN useradd {os.getlogin()} -l -r -u {os.getuid()} -g {os.getgid()} -G sudo 1> /dev/null
            RUN usermod {os.getlogin()} -d {pathlib.Path.home()}
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
class Images(Handler):
    architectures: FrozenSet[str]
    fast: bool
    releases: FrozenSet[str]

    @memoize
    async def _run(self) -> None:
        await asyncio.gather(
            *[Image(architecture=architecture, fast=self.fast, release=release)
              for architecture, release in product(self.architectures, self.releases)])
