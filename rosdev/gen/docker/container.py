from asyncio import get_event_loop
from atools import memoize
from dataclasses import dataclass
import docker
from docker.models.containers import Container as _Container
from frozendict import frozendict
from logging import getLogger
import os
from pathlib import Path
from typing import Mapping

from rosdev.gen.rosdev.config import Config as RosdevConfig
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
    def environment(self) -> frozendict:
        environment = {k: v for k, v in os.environ.items() if 'AWS' in k}
        if self.options.global_setup is not None:
            environment['ROSDEV_GLOBAL_SETUP'] = (
                f'{Path(self.options.global_setup).expanduser().absolute()}'
            )
        if self.options.local_setup is not None:
            environment['ROSDEV_LOCAL_SETUP'] = (
                f'{Path(self.options.local_setup).expanduser().absolute()}'
            )

        if self.options.gui:
            environment['DISPLAY'] = os.environ['DISPLAY']

        if self.options.ccache:
            environment['CC'] = '/usr/lib/ccache/gcc'
            environment['CXX'] = '/usr/lib/ccache/g++'

        #if self.options.sanitizer is not None:
        #    if self.options.sanitizer == 'asan':
        #        environment['LD_PRELOAD'] = (
        #            f'/usr/lib/{get_machine(self.options.architecture)}-linux-gnu/'
        #            f'libasan.so.4'
        #        )
        #    else:
        #        environment['LD_PRELOAD'] = (
        #            f'/usr/lib/{get_machine(self.options.architecture)}-linux-gnu/'
        #            f'lib{self.options.sanitizer}.so.0'
        #        )

        environment = frozendict(environment)

        return environment

    @property
    def volumes(self) -> Mapping[str, str]:
        volumes = {
            **self.options.volumes,
            **RosdevConfig(self.options).volumes,
            f'{Path.cwd()}': f'{Path.cwd()}',
        }

        if self.options.gui:
            volumes['/tmp/.X11-unix'] = '/tmp/.X11-unix'

        return frozendict(volumes)

    @memoize
    async def _main(self) -> None:
        assert Path.home() in Path.cwd().parents

        await Image(self.options)

        client = docker.client.from_env()

        # Make sure only one container with our name exists.
        if self.options.container_name is not None:
            # noinspection PyUnresolvedReferences
            try:
                container = client.containers.get(self.options.container_name)
            except docker.errors.NotFound:
                pass
            else:
                if self.options.replace_named_container:
                    log.info(f'Replacing existing docker container "{self.options.container_name}"')
                    container.remove(force=True)
                else:
                    log.info(f'Found existing docker container "{self.options.container_name}"')
                    return

        log.debug(f'container volumes {self.options.volumes}')
        log.debug(f'container command {self.options.command}')

        container: _Container = client.containers.create(
            auto_remove=self.options.container_name is None,
            command=self.options.command,
            detach=True,
            environment={k: v for k, v in self.environment.items()},
            image=Image(self.options).tag,
            ipc_mode='host',
            name=self.options.container_name,
            ports={port: port for port in self.options.ports},
            privileged=True,  # for X11
            security_opt=['seccomp=unconfined'],  # for GDB
            stdin_open=True,
            tty=True,
            volumes={
                f'{Path(k).expanduser().absolute()}': {
                    'bind': f'{Path(v).expanduser().absolute()}'
                } for k, v in self.volumes.items()
            },
            working_dir=f'{Path.cwd()}',
        )

        if self.options.interactive:
            log.debug(f'Attaching container "{container.name}"')
            os.execlpe('docker', *f'docker start -ai {container.name}'.split(), os.environ)

        log.info(f'Starting container "{container.name}"')

        def attach() -> None:
            for line in container.attach(stdout=True, stderr=True, stream=True):
                print(line)

        await get_event_loop().run_in_executor(None, attach)
        #await exec(f'docker start {container.name}')
