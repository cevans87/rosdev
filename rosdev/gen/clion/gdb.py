from atools import memoize
from dataclasses import dataclass
from frozendict import frozendict
from logging import getLogger
from pathlib import Path

from rosdev.gen.rosdev.config import Config as RosdevConfig
from rosdev.gen.src import Src
from rosdev.util.handler import Handler
from rosdev.util.lookup import get_operating_system
from rosdev.util.subprocess import exec


log = getLogger(__name__)


@memoize
@dataclass(frozen=True)
class Gdb(Handler):

    @property
    def container_path(self) -> str:
        return f'{Path.home()}/.gdbinit'

    @property
    def local_path_base(self) -> str:
        return RosdevConfig(self.options).local_path

    @property
    def local_path(self) -> str:
        return f'{self.local_path_base}/gdbinit'

    @property
    def volumes(self) -> frozendict:
        return frozendict({
            **self.options.volumes,
            self.local_path: self.container_path
        })

    @memoize
    async def _main(self) -> None:
        await Src(self.options)

        await exec(f'mkdir -p {self.local_path_base}')
        with open(self.local_path, 'w') as gdbinit_f_out:
            # FIXME these are two common paths from ci.ro2.org builds. Find a way to
            #  programmatically find the paths, probably through jenkins.
            gdbinit_f_out.write(
                f'set substitute-path '
                f'/home/jenkins-agent/workspace/'
                f'packaging_{get_operating_system(self.options.architecture)}/ws/src/ '
                f'{Src(self.options).local_path}\n'
            )
            gdbinit_f_out.write(
                f'set substitute-path '
                f'/home/rosbuild/ci_scripts/ws/src/ '
                f'{Src(self.options).local_path}\n'
            )

        log.info(f'Gdbinit written to {self.local_path}')
