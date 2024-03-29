from dataclasses import dataclass
import getpass
from logging import getLogger
import os
import platform
from textwrap import dedent, indent

from rosdev.gen.docker.entrypoint_sh import GenDockerEntrypointSh
from rosdev.gen.docker.image_base import GenDockerImageBase
from rosdev.gen.src_base import GenSrcBase
from rosdev.util.atools import memoize, memoize_db
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.path import Path


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerDockerfile(Handler):

    @staticmethod
    @memoize
    async def get_path(options: Options) -> Path:
        path = Path.docker() / 'Dockerfile'

        log.debug(f'{GenDockerDockerfile.__name__} {path = }')

        return path

    @staticmethod
    async def get_from(options: Options) -> str:
        if options.release == 'latest':
            from_tag = 'osrf/ros2:nightly'
        else:
            from_tag = f'{options.architecture}/ros:{options.release}'

        log.debug(f'{GenDockerDockerfile.__name__} {from_tag = }')

        return from_tag

    @staticmethod
    async def get_machine(options: Options) -> str:
        machine = {
            'amd64': 'x86_64',
            'arm32v7': 'armhf',
            'arm64v8': 'aarch64'
        }[options.architecture]

        log.debug(f'{GenDockerDockerfile.__name__} {machine = }')
        
        return machine

    @staticmethod
    async def _get_install_dbgsym_subtext(options: Options) -> str:
        # FIXME the "if in kinetic/melodic" exists to avoid the case where no dbgsym packages
        #  exist. Change this to be resilient against dbgsym packages not existing.
        install_dbgsym_subtext = dedent(fr'''
            apt list --installed "ros-$ROS_DISTRO-*" | \
                grep -o "ros-$ROS_DISTRO-[^/]*" | \
                sed "s/$/-dbgsym/" > \
                a.txt && \
                apt list "ros-$ROS_DISTRO-*-dbgsym" | \
                grep -o "ros-$ROS_DISTRO-[^/]*" > \
                b.txt && \
                grep -Ff a.txt b.txt | \
                xargs -d "\n" -- apt-get install -y && \
                rm a.txt b.txt
        ''').strip() if options.release not in {'kinetic', 'melodic'} else ':'

        return install_dbgsym_subtext

    @staticmethod
    async def _get_download_src_subtext(options: Options) -> str:
        # FIXME the "if in kinetic/melodic" exists to avoid the case where no dbgsym packages
        #  exist. Change this to be resilient against dbgsym packages not existing.
        download_src_subtext = dedent(fr'''
            mkdir -p {await GenSrcBase.get_container_path(options)} && \
                wget https://raw.githubusercontent.com/ros2/ros2/$ROS_DISTRO/ros2.repos && \
                vcs import --input ros2.repos {await GenSrcBase.get_container_path(options)}
        ''').strip() if options.release not in {'kinetic', 'melodic'} else ':'

        return download_src_subtext

    @staticmethod
    async def get_text(options: Options) -> str:
        text = dedent(fr'''
            FROM {await GenDockerDockerfile.get_from(options)}

            # XXX arm32v7 and arm64v8 return error code 100 if we only apt-get update once.
            # see https://github.com/rocker-org/shiny/issues/19#issuecomment-308357402
            RUN apt-get update && \
                apt-get update && \
                {indent(await GenDockerDockerfile._get_install_dbgsym_subtext(options), """
                    """[1:]).strip()} && \
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
                    rsync \
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

            RUN {indent(await GenDockerDockerfile._get_download_src_subtext(options), """
                """[1:]).strip()}
                   
            # We won't use this entrypoint.
            RUN rm /ros_entrypoint.sh

            RUN groupadd -r -g {os.getgid()} {getpass.getuser()} && \
                useradd {getpass.getuser()} -l -r -u {os.getuid()} -g {os.getgid()} -G sudo 1> \
                    /dev/null && \
                usermod {getpass.getuser()} -d {Path.home()} && \
                mkdir -p {Path.home()} && \
                chown {getpass.getuser()}:{getpass.getuser()} {Path.home()} && \
                echo "{getpass.getuser()} ALL=(ALL:ALL) NOPASSWD: ALL" >> /etc/sudoers && \
                touch {Path.home()}/.sudo_as_admin_successful

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
        
        log.debug(f'{GenDockerDockerfile.__name__} {text = }')
        
        return text

    @staticmethod
    @memoize_db(keygen=lambda options: GenDockerImageBase.get_id(options), size=1)
    async def main(options: Options) -> None:
        log.info(f'Creating Dockerfile at {await GenDockerDockerfile.get_path(options)}.')

        (await GenDockerDockerfile.get_path(options)).write_text(
            await GenDockerDockerfile.get_text(options)
        )

        log.info(f'Created Dockerfile at {await GenDockerDockerfile.get_path(options)}.')
