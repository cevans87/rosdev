from atools import memoize
from dataclasses import dataclass
import docker
from frozendict import frozendict
from logging import getLogger
import os
from pathlib import Path

from rosdev.gen.docker.image import Image
from rosdev.util.handler import Handler
from rosdev.util.subprocess import exec


log = getLogger(__name__)


@memoize
@dataclass(frozen=True)
class Container(Handler):

    class Exception(Exception):
        pass

    @property
    def global_rosdev_path(self) -> str:
        return f'{Path.home()}/.rosdev'

    @property
    def local_rosdev_path(self) -> str:
        return f'{os.getcwd()}/.rosdev'

    # FIXME remove this. It's duplicated in Install.local_install_path. Can't import Install here
    #  due to circular import.
    @property
    def local_install_path(self) -> str:
        return f'{self.local_rosdev_path}/install'

    @property
    def environment(self) -> frozendict:
        environment = {k: v for k, v in os.environ.items() if 'AWS' in k}
        # FIXME there are better ways to stop the container from sourcing the ros setup.bash
        if self.options.clean:
            environment['ROSDEV_CLEAN_ENVIRONMENT'] = '1'
        else:
            environment['ROSDEV_DIR'] = os.getcwd()
            environment['ROSDEV_INSTALL_DIR'] = self.local_install_path

        if self.options.gui:
            environment['DISPLAY'] = os.environ['DISPLAY']

        environment = frozendict(environment)

        return environment

    @property
    def volumes(self) -> frozendict:
        assert os.getcwd().startswith(f'{Path.home()}') and (os.getcwd() != f'{Path.home()}'), \
            f'rosdev must be run from a child directory of {Path.home()}'

        volumes = {
            **self.options.volumes,
            self.global_rosdev_path: self.global_rosdev_path,
            os.getcwd(): os.getcwd(),
        }

        if self.options.gui:
            volumes['/tmp/.X11-unix'] = '/tmp/.X11-unix'

        return frozendict(volumes)

    @memoize
    async def _main(self) -> None:
        await Image(self.options)

        assert os.getcwd().startswith(f'{Path.home()}') and os.getcwd() != f'{Path.home()}'

        client = docker.client.from_env()

        # Make sure no other container with our name exists.
        if self.options.name is not None:
            # noinspection PyUnresolvedReferences
            try:
                container = client.containers.get(self.options.name)
            except docker.errors.NotFound:
                pass
            else:
                log.info(f'Removing existing docker container "{self.options.name}"')
                container.remove(force=True)

        container = client.containers.create(
            auto_remove=True,
            command=self.options.command,
            detach=True,
            environment={k: v for k, v in self.environment.items()},
            image=Image(self.options).tag,
            ipc_mode='host',
            name=self.options.name,
            ports={port: port for port in self.options.ports},
            privileged=True,  # for X11
            security_opt=['seccomp=unconfined'],  # for GDB
            stdin_open=True,
            tty=True,
            volumes={k: {'bind': v} for k, v in self.volumes.items()},
            working_dir=os.getcwd(),
        )

        if self.options.interactive:
            log.debug(f'Attaching container "{container.name}"')
            os.execlpe('docker', *f'docker start -ai {container.name}'.split(), os.environ)

        log.info(f'Starting container "{container.name}"')

        await exec(f'docker start {container.name}')
