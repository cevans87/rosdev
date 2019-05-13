from __future__ import annotations
from atools import memoize
from dataclasses import dataclass
from logging import getLogger
import os
from tempfile import TemporaryDirectory

from rosdev.gen.docker.container import Container
from rosdev.util.build_farm import get_ros2_repos
from rosdev.util.handler import Handler
from rosdev.util.subprocess import exec


log = getLogger(__name__)


@memoize
@dataclass(frozen=True)
class Src(Handler):

    @property
    def global_src_path_base(self) -> str:
        return f'{Container(self.options).global_rosdev_path}/src'

    @property
    def global_src_path(self) -> str:
        return f'{self.global_src_path_base}/' \
            f'{self.options.architecture}_{self.options.build_num or self.options.release}'

    @property
    def local_src_symlink_path_base(self) -> str:
        return Container(self.options).local_rosdev_path

    @property
    def local_src_symlink_path(self) -> str:
        return f'{self.local_src_symlink_path_base}/src'

    @property
    def ros_distro(self) -> str:
        if (self.options.build_num is not None) or (self.options.release == 'latest'):
            return 'crystal'

        return self.options.release

    @memoize
    async def _main(self) -> None:
        # TODO split this module into rosdev.gen.global.src and rosdev.gen.local.src
        await self._create_global_src()
        await self._create_local_src_symlink()

    async def _create_global_src(self):
        if os.path.exists(self.global_src_path):
            log.info(f'Found src cached at {self.global_src_path}')
            return

        log.info("Finding src ros2.repos from OSRF build farm")
        ros2_repos = await get_ros2_repos(
            architecture=self.options.architecture,
            build_num=self.options.build_num,
            release=self.options.release,
        )
        with TemporaryDirectory() as temp_dir:
            staging_path = f'{temp_dir}/src'
            ros2_repos_path = f'{temp_dir}/ros2.repos'
            with open(ros2_repos_path, 'w') as ros2_repos_f_out:
                ros2_repos_f_out.write(ros2_repos)

            log.info(f'Staging src at {staging_path}')
            await exec(f'mkdir -p {staging_path}')
            await exec(f'vcs import --input {ros2_repos_path} {staging_path}')

            log.info(f'Caching src at {self.global_src_path}')
            await exec(f'mkdir -p {self.global_src_path_base}')
            await exec(f'mv {staging_path} {self.global_src_path}')

        await exec(f'chmod -R -w {self.global_install_path}')

        log.info(f'Global src at {self.global_src_path}')

    async def _create_local_src_symlink(self) -> None:
        log.info(f'Linking src at {self.local_src_symlink_path}')

        await exec(f'mkdir -p {self.local_src_symlink_path_base}', err_ok=True)
        await exec(f'rm {self.local_src_symlink_path}', err_ok=True)
        await exec(f'ln -s {self.global_src_path} {self.local_src_symlink_path}')

        log.info(f'Linked src at {self.local_src_symlink_path}')
