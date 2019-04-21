from atools import memoize
from dataclasses import dataclass
from frozendict import frozendict
from logging import getLogger
from pathlib import Path

from rosdev.gen.docker.container import Container
from rosdev.gen.install import Install
from rosdev.util.handler import Handler
from rosdev.util.subprocess import exec, get_shell_lines


log = getLogger(__name__)


@memoize
@dataclass(frozen=True)
class Cmake(Handler):

    @property
    def container_pam_environment_path(self) -> str:
        return f'{Path.home()}/.pam_environment'

    @property
    def local_pam_environment_path_base(self) -> str:
        return Container(self.options).local_rosdev_path

    @property
    def local_pam_environment_path(self) -> str:
        return f'{self.local_pam_environment_path_base}/pam_environment'

    @property
    def volumes(self) -> frozendict:
        return frozendict({
            **self.options.volumes,
            self.local_pam_environment_path: self.container_pam_environment_path
        })

    @memoize
    async def _main(self) -> None:
        await Install(self.options)

        await exec(f'mkdir -p {self.local_pam_environment_path_base}')
        with open(self.local_pam_environment_path, 'w') as pam_environment_f_out:
            pam_environment_f_out.write(
                '\n'.join(f'{k}={v}' for k, v in (await self.get_environ()).items())
            )

        log.info(f'PAM environment written to {self.local_pam_environment_path}')

    @memoize
    async def get_environ(self) -> frozendict:
        await Install(self.options)

        lines = await get_shell_lines(
            f'bash -c \'source '
            f'{Install(self.options).local_install_symlink_path}/setup.bash && env\''
        )
        environ = {}
        for line in lines:
            if line:
                k, v = line.split('=', 1)
                if k in {
                    'AMENT_PREFIX_PATH',
                    'CMAKE_PREFIX_PATH',
                    'COLCON_PREFIX_PATH',
                    'LD_LIBRARY_PATH',
                    'PATH',
                    'PYTHONPATH',
                    'ROS_DISTRO',
                    'ROS_PYTHON_VERSION',
                    'ROS_VERSION',
                }:
                    environ[k] = v

        return frozendict(environ)
