from atools import memoize
import docker
from logging import getLogger
import os
from pathlib import Path

from rosdev.gen.docker.images import Image
from rosdev.util.handler import Handler
from rosdev.util.subprocess import exec


log = getLogger(__name__)


@memoize
class Container(Handler):

    class Exception(Exception):
        pass

    @memoize
    async def exit_code(self) -> int:
        await Image(self.options)

        environment = {k: v for k, v in os.environ.items() if 'AWS' in k}
        if self.options.clean:
            environment['ROSDEV_CLEAN_ENVIRONMENT'] = '1'
        else:
            environment['ROSDEV_DIR'] = os.getcwd()
            environment['ROSDEV_INSTALL_DIR'] = (
                f'{os.getcwd()}/.rosdev/{self.options.architecture}/'
                f'{self.options.build_num or self.options.release}'
            )

        volumes = {
            os.getcwd(): {'bind': os.getcwd()},
            str(Path.home()): {'bind': str(Path.home())},
        }
        if self.options.gui:
            environment['DISPLAY'] = os.environ['DISPLAY']
            volumes['/tmp/.X11-unix'] = {'bind': '/tmp/.X11-unix'}

        client = docker.client.from_env()
        container = client.containers.create(
            auto_remove=True,
            command=self.options.command or '/bin/bash',
            detach=True,
            environment=environment,
            image=Image(self.options).tag,
            ipc_mode='host',
            ports={port: port for port in self.options.ports},
            security_opt=['seccomp=unconfined'],
            stdin_open=True,
            tty=True,
            volumes=volumes,
            working_dir=os.getcwd(),
        )

        log.debug(f'attaching to container "{container.name}"')
        if self.options.interactive:
            os.execlpe('docker', *f'docker start -ai {container.name}'.split(), os.environ)

        return await exec(f'docker start -a {container.name}')

    @memoize
    async def must_succeed(self) -> None:
        if await self.exit_code() != 0:
            raise self.Exception('Build failed.')

    @memoize
    async def _main(self) -> None:
        await self.exit_code()
