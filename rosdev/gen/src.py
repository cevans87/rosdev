from atools import memoize
from dataclasses import dataclass, replace
from frozendict import frozendict
from logging import getLogger
import os
from tempfile import TemporaryDirectory
from typing import Mapping

from rosdev.gen.rosdev.config import Config as RosdevConfig
from rosdev.util.build_farm import get_ros2_repos
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.subprocess import exec


log = getLogger(__name__)


@memoize
@dataclass(frozen=True)
class Src(Handler):

    @property
    def container_path_base(self) -> str:
        return RosdevConfig(super().options).container_path

    @property
    def container_path(self) -> str:
        return f'{self.container_path_base}/src'

    @property
    def global_path_base(self) -> str:
        return RosdevConfig(super().options).global_path

    @property
    def global_path(self) -> str:
        return f'{self.global_path_base}/src'

    @property
    def local_path_base(self) -> str:
        return RosdevConfig(super().options).local_path

    @property
    def local_path(self) -> str:
        return f'{self.local_path_base}/src'

    @property
    def options(self) -> Options:
        return replace(
            super().options,
            volumes=self.volumes
        )

    @property
    def volumes(self) -> Mapping[str, str]:
        if super().options.global_setup is None:
            return super().options.volumes
        return frozendict({
            **super().options.volumes,
            self.local_path: self.container_path,
        })

    def __post_init__(self) -> None:
        assert self.options.build_num is not None

    @memoize
    async def _main(self) -> None:
        if self.options.global_setup is None:
            return

        await self._create_global_src()
        await self._create_local_src()

    async def _create_global_src(self):
        if os.path.exists(self.global_path):
            log.info(f'Found src cached at {self.global_path}')
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

            log.info(f'Caching src at {self.global_path}')
            await exec(f'mkdir -p {self.global_path_base}')
            # FIXME this fails if the global path already exists since we recursively made it
            #  read-only
            await exec(f'mv {staging_path} {self.global_path}')

        await exec(f'chmod -R -w {self.global_path}')

        log.info(f'Global src at {self.global_path}')

    async def _create_local_src(self) -> None:
        log.info(f'Linking src at {self.local_path}')

        await exec(f'mkdir -p {self.local_path_base}', err_ok=True)
        await exec(f'rm {self.local_path}', err_ok=True)
        await exec(f'ln -s {self.global_path} {self.local_path}')

        log.info(f'Linked src at {self.local_path}')
