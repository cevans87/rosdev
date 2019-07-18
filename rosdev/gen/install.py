from atools import memoize
from dataclasses import dataclass
from frozendict import frozendict
from logging import getLogger
import os
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import Mapping

from rosdev.gen.rosdev.config import Config as RosdevConfig
from rosdev.util.build_farm import get_artifacts_url, get_build_num
from rosdev.util.handler import Handler
from rosdev.util.subprocess import exec


log = getLogger(__name__)


@memoize
@dataclass(frozen=True)
class Install(Handler):

    @property
    def container_path_base(self) -> str:
        return RosdevConfig(self.options).container_path

    @property
    def container_path(self) -> str:
        return f'{self.container_path_base}/install'

    @property
    def global_path_base(self) -> str:
        return (
            f'{RosdevConfig(super().options).global_path}/{super().options.architecture}/'
            f'{super().options.build_num}'
        )

    @property
    def global_path(self) -> str:
        return f'{self.global_path_base}/install'

    @property
    def local_path_base(self) -> str:
        return RosdevConfig(super().options).local_path

    @property
    def local_path(self) -> str:
        return f'{self.local_path_base}/install'

    @property
    def volumes(self) -> Mapping[str, str]:
        if self.options.global_setup is None:
            return self.options.volumes

        return frozendict({
            **self.options.volumes,
            self.local_path: self.container_path,
        })

    @memoize
    async def get_global_path(self) -> str:

        build_num = self.options.build_num
        if self.options.build_num is not None:
            build_num = self.options.build_num
        elif self.options.pull_install:
            build_num = await get_build_num(
                architecture=self.options.architecture, release=self.options.release
            )
        # TODO change to walrus operator in py38
        elif self.options.release == 'latest':
            paths = sorted(Path(self.global_path_base).glob(f'{self.options.architecture}_*'))
            if paths:
                build_num = int(paths[-1].parts[-1].lstrip(f'{self.options.architecture}_'))
            else:
                build_num = await get_build_num(
                    architecture=self.options.architecture, release=self.options.release
                )

        return f'{self.global_path_base}/{self.options.architecture}_{build_num}'

    def __post_init__(self) -> None:
        assert self.options.build_num is not None

    @memoize
    async def _main(self) -> None:
        if self.options.global_setup is None:
            return

        await self._create_global_install()
        await self._create_local_install()

    async def _create_global_install(self):
        if os.path.exists(self.global_path):
            log.info(f'Found install cached at {self.global_path}')
            return

        log.info("Finding artifacts url from OSRF build farm")
        artifacts_url = await get_artifacts_url(
            architecture=self.options.architecture,
            build_num=self.options.build_num,
            release=self.options.release,
        )

        log.info("Installing from OSRF build farm")
        with TemporaryDirectory() as temp_dir:
            staging_path = f'{temp_dir}/install'
            artifacts_path = f'{temp_dir}/artifacts.tar.bz2'

            log.info(f'Downloading install artifacts to {artifacts_path}')
            await exec(f'wget {artifacts_url} -O {artifacts_path}')

            log.info(f'Staging install at {staging_path}')
            await exec(f'mkdir -p {staging_path}')
            await exec(f'tar -xf {artifacts_path} -C {staging_path} --strip-components 1')

            log.info(f'Caching install at {self.global_path}')
            await exec(f'mkdir -p {self.global_path_base}')
            # FIXME this fails if the global path already exists since we recursively made it
            #  read-only
            await exec(f'mv {staging_path} {self.global_path}')

        await exec(f'chmod -R -w {self.global_path}')

        log.info(f'Global install at {self.global_path}')

    async def _create_local_install(self) -> None:
        log.info(f'Linking install at {self.local_path}')

        await exec(f'mkdir -p {self.local_path_base}', err_ok=True)
        await exec(f'rm {self.local_path}', err_ok=True)
        await exec(f'ln -s {self.global_path} {self.local_path}')

        log.info(f'Linked install at {self.local_path}')
