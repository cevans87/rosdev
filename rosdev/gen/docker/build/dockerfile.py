from dataclasses import dataclass, field
from logging import getLogger
import os
from pathlib import Path
import platform
from textwrap import dedent
from typing import Tuple, Type

from rosdev.gen.architecture import GenArchitecture
from rosdev.gen.base import GenBase
from rosdev.gen.docker.build.context import GenDockerBuildContext
from rosdev.gen.docker.entrypoint_sh import GenDockerEntrypointSh
from rosdev.gen.home import GenHome
from rosdev.gen.ros.build_num import GenRosBuildNum
from rosdev.gen.ros.release import GenRosRelease
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerBuildDockerfile(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenArchitecture,
        GenBase,
        GenDockerBuildContext,
        GenDockerEntrypointSh,
        GenHome,
        GenRosBuildNum,
        GenRosRelease,
    ))

    @classmethod
    def get_qemu_path(cls, options: Options) -> str:
        return f'/usr/bin/qemu-{options.machine}-static'

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
                   
            # We won't use this entrypoint. We mount our entrypoint when we start the container.
            RUN rm /ros_entrypoint.sh

            RUN groupadd -r -g {os.getgid()} {os.getlogin()}
            RUN useradd {os.getlogin()} -l -r -u {os.getuid()} -g {os.getgid()} -G sudo 1> /dev/null
            RUN usermod {os.getlogin()} -d {options.home_container_path}
            RUN mkdir -p {options.home_container_path}
            RUN chown {os.getlogin()}:{os.getlogin()} {options.home_container_path}
            RUN echo "{os.getlogin()} ALL=(ALL:ALL) NOPASSWD: ALL" >> /etc/sudoers
            # Stops annoying message about being a sudoer when you first log in.
            RUN touch {options.home_container_path}/.sudo_as_admin_successful
            
            # Allow anonymous ssh login
            RUN sed -i -re 's/^{os.getlogin()}:[^:]+:/{os.getlogin()}::/' /etc/passwd /etc/shadow
            RUN echo 'sshd : ALL : allow' >> /etc/hosts.allow
            
            # TODO specify user_envfile to load a different .pam_environment
            run sed -i "s/readenv=1 envfile/readenv=1 user_readenv=1 envfile/g" /etc/pam.d/login

            USER {os.getlogin()}

            ENTRYPOINT ["{options.docker_entrypoint_sh_container_path}"]
        ''').lstrip()

    @classmethod
    async def main(cls, options: Options) -> None:
        log.info(f'Creating Dockerfile in {options.docker_build_context_workspace_path}')
        with open(Path(options.docker_build_context_workspace_path, 'Dockerfile'), 'w') as f_out:
            f_out.write(cls.get_dockerfile_contents(options))
        log.info(f'Created Dockerfile in {options.docker_build_context_workspace_path}')
