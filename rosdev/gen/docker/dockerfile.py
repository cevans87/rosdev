from dataclasses import dataclass, field
from logging import getLogger
import os
import platform
from textwrap import dedent
from typing import Tuple, Type

from rosdev.gen.base import GenBase
from rosdev.gen.host import GenHost
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerDockerfile(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenBase,
        GenHost,
    ))

    @classmethod
    def get_qemu_path(cls, options: Options) -> str:
        return f'/usr/bin/qemu-{options.machine}-static'

    @classmethod
    def get_dockerfile_contents(cls, options: Options) -> str:
        return dedent(fr'''
            FROM {options.docker_image_base_tag}

            # qemu static binaries
            #{f'VOLUME {cls.get_qemu_path(options)}' if platform.machine() != options.machine
             and platform.system() != 'Darwin' else '# not needed'}
            
            # XXX arm32v8 and arm64v8 return error code 100 if we only apt-get update once.
            # see https://github.com/rocker-org/shiny/issues/19#issuecomment-308357402
            RUN apt-get update && apt-get update
            
            # FIXME this is insanely slow. The apt-cache search part is the primary culprit.
            # Install debug symbols for all installed ros packages.
            #RUN apt list --installed "ros-$ROS_DISTRO*" | \
            #    grep -o "ros-$ROS_DISTRO-[^/]*" | \
            #    sed "s/$/-dbgsym/" | \
            #    xargs -L1 apt-cache search | \
            #    grep -o "ros-$ROS_DISTRO-.*-dbgsym" | \
            #    xargs -d "\n" -- apt-get install -y

            RUN apt-get install -y \
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
                pytest \
                pytest-cov \
                pytest-repeat \
                pytest-rerunfailures \
                pytest-runner \
                rosdep \
                setuptools \
                vcstool
                   
            # We won't use this entrypoint. We mount our entrypoint when we start the container.
            RUN rm /ros_entrypoint.sh

            RUN groupadd -r -g {os.getgid()} {os.getlogin()}
            RUN useradd {os.getlogin()} -l -r -u {os.getuid()} -g {os.getgid()} -G sudo 1> /dev/null
            RUN usermod {os.getlogin()} -d {options.home_path}
            RUN mkdir -p {options.home_path}
            RUN chown {os.getlogin()}:{os.getlogin()} {options.home_path}
            RUN echo "{os.getlogin()} ALL=(ALL:ALL) NOPASSWD: ALL" >> /etc/sudoers
            # Stops annoying message about being a sudoer when you first log in.
            RUN touch {options.home_path}/.sudo_as_admin_successful
            
            # Allow anonymous ssh login
            RUN sed -i -re 's/^{os.getlogin()}:[^:]+:/{os.getlogin()}::/' /etc/passwd /etc/shadow
            RUN echo 'sshd : ALL : allow' >> /etc/hosts.allow
            
            # TODO specify user_envfile to load a different .pam_environment
            run sed -i "s/readenv=1 envfile/readenv=1 user_readenv=1 envfile/g" /etc/pam.d/login

            USER {os.getlogin()}
            
            COPY {options.docker_entrypoint_sh_host_path.parts[-1]} \
                {options.docker_entrypoint_sh_container_path}

            ENTRYPOINT ["{options.docker_entrypoint_sh_container_path}"]
        ''').lstrip()

    @classmethod
    async def main(cls, options: Options) -> None:
        log.info(f'Creating Dockerfile at {options.docker_dockerfile_path}')
        GenHost.write_text(
            data=cls.get_dockerfile_contents(options),
            options=options,
            path=options.docker_dockerfile_path,
        )
        log.info(f'Created Dockerfile at {options.docker_dockerfile_path}')
