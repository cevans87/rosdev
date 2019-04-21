from __future__ import annotations
from atools import memoize
from dataclasses import dataclass
from frozendict import frozendict
from jenkins import Jenkins
from logging import getLogger
import os
import re
from tempfile import TemporaryDirectory
from typing import List

from rosdev.gen.docker.container import Container
from rosdev.util.handler import Handler
from rosdev.util.lookup import get_build_num, get_operating_system
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
    def global_src_ro_mnt_path(self) -> str:
        return f'{self.global_src_path}_ro_mnt'

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
        await self._create_global_src()
        await self._create_global_src_ro_mnt()
        await self._create_local_src_symlink()

    async def _create_global_src(self):
        if os.path.exists(self.global_src_path):
            log.info(f'Found src cached at {self.global_src_path}')
            return

        if self.options.build_num is not None:
            build_num = self.options.build_num
        else:
            build_num = get_build_num(self.options.architecture, self.options.release)

        log.info("Finding src from OSRF build farm")
        # FIXME blocking call in async context
        lines = [
            line.rstrip() for line in
            Jenkins('https://ci.ros2.org').get_build_console_output(
                f'packaging_{get_operating_system(self.options.architecture)}', build_num
            ).split('\n')
        ]

        remaining_lines = iter(lines)
        for line in remaining_lines:
            if re.match(r'^# BEGIN SUBSECTION: vcs export --exact$', line) is not None:
                break
        for line in remaining_lines:
            if re.match(r'^repositories:$', line) is not None:
                break
        ros2_repos_lines: List[str] = ['repositories:']
        for line in remaining_lines:
            if re.match(r'^# END SUBSECTION$', line) is not None:
                break
            elif re.match(r'\s+\S+:.*$', line) is not None:
                ros2_repos_lines.append(line)

        with TemporaryDirectory() as temp_dir:
            staging_path = f'{temp_dir}/src'
            ros2_repos_path = f'{temp_dir}/ros2.repos'
            with open(ros2_repos_path, 'w') as ros2_repos_f_out:
                ros2_repos_f_out.write('\n'.join(ros2_repos_lines))

            log.info(f'Staging src at {staging_path}')
            await exec(f'mkdir -p {staging_path}')
            await exec(f'vcs import --input {ros2_repos_path} {staging_path}')

            log.info(f'Caching src at {self.global_src_path}')
            await exec(f'mkdir -p {self.global_src_path_base}')
            await exec(f'mv {staging_path} {self.global_src_path}')

        log.info(f'Src cached at {self.global_src_path}')

    async def _create_global_src_ro_mnt(self) -> None:
        log.info(f'Binding read-only src cache at {self.global_src_ro_mnt_path}')
        if os.path.exists(self.global_src_ro_mnt_path):
            await exec(f'fusermount -u {self.global_src_ro_mnt_path}', err_ok=True)

        await exec(f'mkdir -p {self.global_src_ro_mnt_path}', err_ok=True)
        await exec(
            f'bindfs --no-allow-other --perms=a-w '
            f'{self.global_src_path} {self.global_src_ro_mnt_path}'
        )

        log.info(f'Bound read-only src cache at {self.global_src_ro_mnt_path}')

    async def _create_local_src_symlink(self) -> None:
        log.info(f'Linking src at {self.local_src_symlink_path}')

        await exec(f'mkdir -p {self.local_src_symlink_path_base}', err_ok=True)
        await exec(f'rm {self.local_src_symlink_path}', err_ok=True)
        await exec(f'ln -s {self.global_src_ro_mnt_path} {self.local_src_symlink_path}')

        log.info(f'Linked src at {self.local_src_symlink_path}')
