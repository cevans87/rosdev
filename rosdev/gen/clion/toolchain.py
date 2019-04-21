from asyncio import gather
from atools import memoize
from dataclasses import dataclass
from frozendict import frozendict
from logging import getLogger
import os
from pathlib import Path
from textwrap import dedent

from rosdev.gen.clion.cmake import Cmake
from rosdev.gen.clion.gdb import Gdb
from rosdev.gen.docker.container import Container
from rosdev.gen.install import Install
from rosdev.gen.src import Src
from rosdev.util.handler import Handler
from rosdev.util.subprocess import exec


log = getLogger(__name__)


@memoize
@dataclass(frozen=True)
class Toolchain(Handler):

    @property
    def volumes(self) -> frozendict:
        return frozendict({
            **self.options.volumes,
            **Cmake(self.options).volumes,
            **Gdb(self.options).volumes,
            f'{Path.home()}/.ssh': f'{Path.home()}/.ssh',
        })

    @memoize
    async def _main(self) -> None:
        await gather(
            Cmake(self.options),
            Gdb(self.options),
            Install(self.options),
            Src(self.options)
        )
        await gather(
            exec(f'ssh-keygen -f "{Path.home()}/.ssh/known_hosts" -R "localhost"', err_ok=True),
            Container(
                options=self.options(
                    command='bash -c "sudo service ssh start && sleep infinity"',
                    ports=frozenset({22}),
                    name='rosdev_gen_clion_toolchain',
                    volumes=self.volumes,
                ),
            )
        )

        log.info('Toolchain running at localhost:22')
        log.info(dedent(f'''
            Toolchain Setup (Do this once on your computer)
                1. Go to Settings | Build, Execution, Deployment > Toolchains
                2. Select "+" to create a new toolchain
                3. Name the new toolchain anything you'd like
                4. Change toolchain type from "System" to "Remote Host"
                5. Credentials: press "shift+enter" or click icon to bring up sub menu
                    Host: localhost
                    Port: 22
                    User name: {os.getlogin()}
                    Authentication type: Key pair
                    Private key file: usually {Path.home()}/.ssh/id_rsa
                    Passphrase: usually blank
                6. Make the new toolchain the default by dragging it above the current default
        '''))
