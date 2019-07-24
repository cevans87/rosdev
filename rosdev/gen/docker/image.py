import asyncio
from dataclasses import dataclass, field, replace
import docker
from logging import getLogger
import os
from pathlib import Path
import platform
from tempfile import TemporaryDirectory
from textwrap import dedent
from typing import Tuple, Type

from rosdev.gen.architecture import GenArchitecture
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerImage(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenArchitecture,
    ))

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        docker_image_base_tag = options.docker_image_base_tag
        if docker_image_base_tag is None:
            if options.ros_release != 'latest':
                docker_image_base_tag = (
                    f'{cls.get_profile(options)}/ros:{options.ros_release}-{options.flavor}'
                )
            elif cls.get_profile(options) != 'amd64':
                docker_image_base_tag = '{cls.get_profile(options)}/ros:crystal-{options.flavor}'
            else:
                docker_image_base_tag = 'osrf/ros2:nightly'
                
        docker_image_tag = options.docker_image_tag
        if docker_image_tag is None:
            docker_image_tag = f'{docker_image_base_tag}-dev'

        return replace(
            options,
            docker_image_base_tag=docker_image_base_tag,
            docker_image_tag=docker_image_tag,
        )

    @classmethod
    def get_profile(cls, options: Options) -> str:
        if options.flavor in {'desktop', 'desktop-full'}:
            return 'osrf'
        else:
            return options.architecture

    @classmethod
    def get_qemu_path(cls, options: Options) -> str:
        return f'/usr/bin/qemu-{options.machine}-static'

    @classmethod
    def get_rosdev_entrypoint_sh_contents(cls, options: Options) -> str:
        return dedent(fr'''
            #!/bin/bash
            set -e
            
            if [ ! -z ${{ROSDEV_ROS_SETUP_UNDERLAY_CONTAINER_PATH+x}} ]; then \
               source "$ROSDEV_ROS_SETUP_UNDERLAY_CONTAINER_PATH"
            fi
            
            if [ ! -z ${{ROSDEV_ROS_SETUP_OVERLAY_CONTAINER_PATH+x}} ]; then \
               source "$ROSDEV_ROS_SETUP_OVERLAY_CONTAINER_PATH"
            fi
            
            exec "$@"
        ''').lstrip()

    @classmethod
    def get_dockerfile_contents(cls, options: Options) -> str:
        return dedent(fr'''
            FROM {options.docker_image_base_tag}

            # qemu static binaries
            {f'VOLUME {cls.get_qemu_path(options)}' if platform.machine() != options.machine
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
                git \
                liblog4cxx-dev \
                openssh-server \
                python3-pip \
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
                rosdep \
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

    @classmethod
    async def main(cls, options: Options) -> None:
        def main_internal() -> None:
            log.info(
                f'Creating docker image {options.docker_image_tag} '
                f'from {options.docker_image_base_tag}'
            )
            with TemporaryDirectory() as tempdir_path:
                with open(f'{tempdir_path}/Dockerfile', 'w') as dockerfile_f_out:
                    dockerfile_f_out.write(cls.get_dockerfile_contents(options))
                with open(f'{tempdir_path}/rosdev_entrypoint.sh', 'w') as entrypoint_f_out:
                    entrypoint_f_out.write(cls.get_rosdev_entrypoint_sh_contents(options))
                with open(f'{Path.home()}/.ssh/id_rsa.pub', 'r') as id_rsa_f_in, \
                        open(f'{tempdir_path}/id_rsa.pub', 'w') as id_rsa_f_out:
                    id_rsa_f_out.write(id_rsa_f_in.read())

                client = docker.client.from_env()
                client.images.build(
                    path=tempdir_path,
                    pull=options.pull_docker_image,
                    rm=True,
                    tag=options.docker_image_tag,
                )
            log.info(
                f'Created docker image {options.docker_image_tag} '
                f'from {options.docker_image_base_tag}'
            )

        await asyncio.get_event_loop().run_in_executor(None, main_internal)
