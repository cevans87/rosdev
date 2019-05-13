from atools import memoize
from dataclasses import dataclass
from logging import getLogger
import os
from tempfile import TemporaryDirectory

from rosdev.gen.docker.container import Container
from rosdev.util.handler import Handler
from rosdev.util.lookup import get_build_num, get_machine, get_operating_system
from rosdev.util.subprocess import exec


log = getLogger(__name__)


@memoize
@dataclass(frozen=True)
class Install(Handler):

    @property
    def global_install_path_base(self) -> str:
        return f'{Container(self.options).global_rosdev_path}/install'

    @property
    def global_install_path(self) -> str:
        return f'{self.global_install_path_base}/' \
            f'{self.options.architecture}_{self.options.build_num or self.options.release}'

    @property
    def local_install_symlink_path_base(self) -> str:
        return Container(self.options).local_rosdev_path

    @property
    def local_install_symlink_path(self) -> str:
        return f'{self.local_install_symlink_path_base}/install'

    @property
    def ros_distro(self) -> str:
        if (self.options.build_num is not None) or (self.options.release == 'latest'):
            return 'crystal'

        return self.options.release

    @memoize
    async def _main(self) -> None:
        # TODO split this module into rosdev.gen.global.install and rosdev.gen.local.install
        await self._create_global_install()
        await self._create_local_install_symlink()

    async def _create_global_install(self):
        if os.path.exists(self.global_install_path):
            log.info(f'Found install cached at {self.global_install_path}')
            return

        if self.options.build_num is not None or self.options.release != 'latest':
            await self._create_global_install_from_osrf_build_farm()
        else:
            await self._create_global_install_from_container()

        await exec(f'chmod -R -w {self.global_install_path}')

        log.info(f'Global nstall at {self.global_install_path}')

    async def _create_global_install_from_container(self) -> None:
        log.info(
            f'Installing from docker image {self.options.release} to {self.global_install_path}')
        await exec(f'mkdir -p {self.global_install_path}')
        await Container(
            self.options(
                clean=True,
                command=f'cp -r /opt/ros/{self.ros_distro} {self.global_install_path}',
                interactive=False,
            )
        )

    async def _create_global_install_from_osrf_build_farm(self) -> None:
        log.info("Installing from OSRF build farm")
        if self.options.build_num is not None:
            build_num = self.options.build_num
        else:
            build_num = get_build_num(self.options.architecture, self.options.release)

        with TemporaryDirectory() as temp_dir:
            staging_path = f'{temp_dir}/install'
            artifacts_path = f'{temp_dir}/artifacts.tar.bz2'

            log.info(f'Downloading install artifacts at {artifacts_path}')
            await exec(
                f'wget https://ci.ros2.org/view/packaging/job/'
                f'packaging_{get_operating_system(self.options.architecture)}/{build_num}/artifact/'
                f'ws/ros2-package-linux-{get_machine(self.options.architecture)}.tar.bz2 '
                f'-O {artifacts_path}'
            )

            log.info(f'Staging install at {staging_path}')
            await exec(f'mkdir -p {staging_path}')
            await exec(f'tar -xf {artifacts_path} -C {staging_path} --strip-components 1')

            log.info(f'Caching install at {self.global_install_path}')
            await exec(f'mkdir -p {self.global_install_path_base}')
            await exec(f'mv {staging_path} {self.global_install_path}')

    async def _create_local_install_symlink(self) -> None:
        log.info(f'Linking install at {self.local_install_symlink_path}')

        await exec(f'mkdir -p {self.local_install_symlink_path_base}', err_ok=True)
        await exec(f'rm {self.local_install_symlink_path}', err_ok=True)
        await exec(f'ln -s {self.global_install_path} {self.local_install_symlink_path}')

        log.info(f'Linked install at {self.local_install_symlink_path}')
