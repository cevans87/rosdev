from __future__ import annotations
import asyncio
from atools import memoize
import docker
from logging import getLogger
import os
from pathlib import Path
import platform
from tempfile import TemporaryDirectory
from textwrap import dedent

from rosdev.gen.install import Install
from rosdev.util.handler import Handler
from rosdev.util.lookup import get_machine


log = getLogger(__name__)


@memoize
class Image(Handler):

    class Exception(Exception):
        pass

    @property
    def cwd(self) -> str:
        return f'{Path.cwd()}'

    @property
    def profile(self) -> str:
        if self.options.flavor in {'desktop', 'desktop-full'}:
            return 'osrf'
        else:
            return self.options.architecture

    @property
    def base_tag(self) -> str:
        if self.options.release != 'latest':
            return f'{self.profile}/ros:{self.options.release}-{self.options.flavor}'
        elif self.profile != 'amd64':
            return f'{self.profile}/ros:crystal-{self.options.flavor}'
        else:
            return f'osrf/ros2:nightly'

    @property
    def tag(self) -> str:
        return f'{self.base_tag}-dev'

    @property
    def machine(self) -> str:
        return get_machine(self.options.architecture)

    @property
    def qemu_path(self) -> str:
        return f'/usr/bin/qemu-{self.machine}-static'

    @property
    def rosdev_entrypoint_sh_contents(self) -> str:
        # TODO improve the way we're executing this entrypoint. Make this global entrypoint
        # execute a local entrypoint, which we will volume mount from the local rosdev workspace
        return dedent(fr'''
            #!/bin/bash
            set -e
            
            if [ ! -z ${{ROSDEV_GLOBAL_SETUP+x}} ]; then \
               source "$ROSDEV_GLOBAL_SETUP"
            fi
            
            if [ ! -z ${{ROSDEV_LOCAL_SETUP+x}} ]; then \
               source "$ROSDEV_LOCAL_SETUP"
            fi
            
            exec "$@"
        ''').lstrip()

    @property
    def dockerfile_contents(self) -> str:
        # FIXME see how hard it is to host this image on Dockerhub. It takes a while to build.
        # FIXME allow user to specify additional apt and pip3 packages
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
                ccache \
                cmake \
                coreutils \
                curl \
                gdb \
                gdbserver \
                git \
                liblog4cxx-dev \
                openssh-server \
                python3-pip \
                python3-colcon-common-extensions \
                python3-pip \
                python-rosdep \
                python3-vcstool \
                sudo \
                wget
                
            RUN apt-get install --no-install-recommends -y \
                libasio-dev \
                libtinyxml2-dev \
                && apt-get clean

            RUN python3 -m pip install -U \
                argcomplete \
                colcon-core \
                colcon-common-extensions \
                colcon-mixin \
                flake8 \
                flake8-blind-except \
                flake8-builtins \
                flake8-class-newline \
                flake8-comprehensions \
                flake8-deprecated \
                flake8-docstrings \
                flake8-import-order \
                flake8-quotes \
                git+https://github.com/lark-parser/lark.git@0.7d \
                numpy \
                pytest \
                pytest-cov \
                pytest-repeat \
                pytest-rerunfailures \
                pytest-runner \
                setuptools \
                vcstool
                
            RUN colcon mixin add default \
                https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
            RUN colcon mixin update default
                   
            RUN rm /ros_entrypoint.sh
            COPY rosdev_entrypoint.sh /
            RUN chmod +x /rosdev_entrypoint.sh

            RUN groupadd -r -g {os.getgid()} {os.getlogin()}
            RUN useradd {os.getlogin()} -l -r -u {os.getuid()} -g {os.getgid()} -G sudo 1> /dev/null
            RUN usermod {os.getlogin()} -d {Path.home()}
            RUN mkdir -p {Path.home()}
            RUN chown {os.getlogin()}:{os.getlogin()} {Path.home()}
            RUN echo "{os.getlogin()} ALL=(ALL:ALL) NOPASSWD: ALL" >> /etc/sudoers
            RUN touch {Path.home()}/.sudo_as_admin_successful
            
            # Allow anonymous ssh login
            RUN sed -i -re 's/^{os.getlogin()}:[^:]+:/{os.getlogin()}::/' /etc/passwd /etc/shadow
            RUN echo 'sshd : ALL : allow' >> /etc/hosts.allow
            
            # TODO specify user_envfile to load a different .pam_environment
            run sed -i "s/readenv=1 envfile/readenv=1 user_readenv=1 envfile/g" /etc/pam.d/login

            USER {os.getlogin()}

            ENTRYPOINT ["/rosdev_entrypoint.sh"]
            CMD bash
        ''').lstrip()

    @memoize
    async def _main(self) -> None:
        def _main_internal() -> None:
            with TemporaryDirectory() as tempdir_path:
                with open(f'{tempdir_path}/Dockerfile', 'w') as dockerfile_f_out:
                    dockerfile_f_out.write(self.dockerfile_contents)
                with open(f'{tempdir_path}/rosdev_entrypoint.sh', 'w') as entrypoint_f_out:
                    entrypoint_f_out.write(self.rosdev_entrypoint_sh_contents)
                with open(f'{Path.home()}/.ssh/id_rsa.pub', 'r') as id_rsa_f_in, \
                        open(f'{tempdir_path}/id_rsa.pub', 'w') as id_rsa_f_out:
                    id_rsa_f_out.write(id_rsa_f_in.read())

                client = docker.client.from_env()
                client.images.build(
                    path=tempdir_path,
                    pull=self.options.pull,
                    rm=True,
                    tag=self.tag,
                )

        log.info(f'Creating docker image {self.tag} from {self.base_tag}')
        await asyncio.get_event_loop().run_in_executor(None, _main_internal)
        log.info(f'Created docker image {self.tag} from {self.base_tag}')
