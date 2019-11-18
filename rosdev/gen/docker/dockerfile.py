from atools import memoize
from dataclasses import dataclass
import getpass
from logging import getLogger
import os
from pathlib import Path
import platform
from textwrap import dedent

from rosdev.gen.docker.entrypoint_sh import GenDockerEntrypointSh
from rosdev.gen.home import GenHome
from rosdev.gen.host import GenHost
from rosdev.gen.rosdev.home import GenRosdevHome
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerDockerfile(Handler):

    @classmethod
    @memoize
    async def get_path(cls, options: Options) -> Path:
        path = await GenRosdevHome.get_path(options) / 'docker' / 'Dockerfile'

        log.debug(f'{cls.__name__} {path = }')

        return path

    @classmethod
    @memoize
    async def get_from(cls, options: Options) -> str:
        if options.release == 'latest':
            from_tag = 'osrf/ros2:nightly'
        else:
            from_tag = f'{options.architecture}/ros:{options.release}'

        log.debug(f'{cls.__name__} {from_tag = }')

        return from_tag

    @classmethod
    def get_qemu_path(cls, options: Options) -> str:
        return f'/usr/bin/qemu-{options.machine}-static'

    @classmethod
    async def get_text(cls, options: Options) -> str:
        return dedent(fr'''
            FROM {await cls.get_from(options)}

            # qemu static binaries
            #{f'VOLUME {cls.get_qemu_path(options)}' if platform.machine() != options.machine
             and platform.system() != 'Darwin' else '# not needed'}
            
            # XXX arm32v7 and arm64v8 return error code 100 if we only apt-get update once.
            # see https://github.com/rocker-org/shiny/issues/19#issuecomment-308357402
            RUN apt-get update && apt-get update

            # Install debug symbols for all installed ros packages.
            RUN apt list --installed "ros-$ROS_DISTRO-*" | \
                    grep -o "ros-$ROS_DISTRO-[^/]*" | \
                    sed "s/$/-dbgsym/" > \
                    a.txt && \
                    apt list "ros-$ROS_DISTRO-*-dbgsym" | \
                    grep -o "ros-$ROS_DISTRO-[^/]*" > \
                    b.txt && \
                    grep -Ff a.txt b.txt | \
                    xargs -d "\n" -- apt-get install -y && \
                    rm a.txt b.txt && \
                apt-get install -y \
                    build-essential \
                    ccache \
                    cmake \
                    coreutils \
                    curl \
                    gdb \
                    git \
                    liblog4cxx-dev \
                    openssh-server \
                    python3-pip \
                    sudo \
                    wget && \
                apt-get install --no-install-recommends -y \
                    libasio-dev \
                    libtinyxml2-dev && \
                apt-get clean

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
                pytest \
                pytest-cov \
                pytest-repeat \
                pytest-rerunfailures \
                pytest-runner \
                rosdep \
                setuptools \
                vcstool
                   
            # We won't use this entrypoint.
            RUN rm /ros_entrypoint.sh

            RUN groupadd -r -g {os.getgid()} {getpass.getuser()} && \
                useradd {getpass.getuser()} -l -r -u {os.getuid()} -g {os.getgid()} -G sudo 1> \
                    /dev/null && \
                usermod {getpass.getuser()} -d {await GenHome.get_path(options)} && \
                mkdir -p {await GenHome.get_path(options)} && \
                chown {getpass.getuser()}:{getpass.getuser()} {await GenHome.get_path(options)} && \
                echo "{getpass.getuser()} ALL=(ALL:ALL) NOPASSWD: ALL" >> /etc/sudoers && \
                touch {await GenHome.get_path(options)}/.sudo_as_admin_successful

            # Allow anonymous ssh login
            RUN sed -i -re 's/^{getpass.getuser()}:[^:]+:/{getpass.getuser()}::/' \
                    /etc/passwd /etc/shadow && \
                echo 'sshd : ALL : allow' >> /etc/hosts.allow
            
            # TODO specify user_envfile to load a different .pam_environment
            run sed -i "s/readenv=1 envfile/readenv=1 user_readenv=1 envfile/g" /etc/pam.d/login

            USER {getpass.getuser()}
            
            COPY {(await GenDockerEntrypointSh.get_path(options)).parts[-1]} \
                {await GenDockerEntrypointSh.get_container_path(options)}

            ENTRYPOINT ["{await GenDockerEntrypointSh.get_container_path(options)}"]
        ''').lstrip()

    @classmethod
    async def main(cls, options: Options) -> None:
        if (
                (not options.docker_image_pull) and
                (not options.docker_image_replace) and
                (await cls.get_path(options)).exists()
        ):
            return

        log.info(f'Creating Dockerfile at {await cls.get_path(options)}.')
        GenHost.write_text(
            data=await cls.get_text(options),
            options=options,
            path=await cls.get_path(options),
        )
        log.info(f'Created Dockerfile at {await cls.get_path(options)}.')
