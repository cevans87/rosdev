from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from pathlib import Path

from rosdev.gen.docker.container import GenDockerContainer
from rosdev.gen.home import GenHome
from rosdev.gen.host import GenHost
from rosdev.gen.rosdev.workspace import GenRosdevWorkspace
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerGdbinit(Handler):

    @classmethod
    @memoize
    async def get_path(cls, options: Options) -> Path:
        path = await GenRosdevWorkspace.get_path(options) / 'gdbinit'
        
        log.debug(f'{cls.__name__} {path = }')
        
        return path

    @classmethod
    @memoize
    async def get_symlink_path(cls, options: Options) -> Path:
        symlink_path = await GenHome.get_path(options) / '.gdbinit'

        log.debug(f'{cls.__name__} {symlink_path = }')

        return symlink_path

    @classmethod
    async def main(cls, options: Options) -> None:
        log.info(f'Creating docker_gdbinit')
        GenHost.write_text(
            # FIXME these are two common paths from ci.ro2.org builds. Find a way to
            #  programmatically find the paths, probably through jenkins.
            data=(
                f'set directories {options.src_symlink_path / "ros2"}'
                f'set substitute-path '
                f'/home/jenkins-agent/workspace/'
                f'packaging_{options.operating_system}/ws/src/ '
                f'{options.src_symlink_path}\n'
                f'set substitute-path '
                f'/home/rosbuild/ci_scripts/ws/src/ '
                f'{options.src_symlink_path}\n'
            ),
            options=options,
            path=options.docker_gdbinit_path,
        )
        await GenDockerContainer.execute(
            commmand=f'ln -s {await cls.get_path(options)} {await cls.get_symlink_path(options)}',
            options=options,
        )

        log.info(f'Created docker_gdbinit')
