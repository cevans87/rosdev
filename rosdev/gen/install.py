from __future__ import annotations
from asyncio import get_event_loop
from atools import memoize
from dataclasses import dataclass
from functools import partial
from logging import getLogger
import os
import pathlib
import shutil
from tempfile import TemporaryDirectory
from typing import Generator, Optional

from rosdev.gen.docker.container import container
from rosdev.util.lookup import get_machine, get_operating_system
from rosdev.util.subprocess import exec


log = getLogger(__package__)


@dataclass(frozen=True)
class _Install:
    architecture: str
    build_num: Optional[str]
    release: str

    @property
    @memoize
    def cwd(self) -> str:
        return f'{pathlib.Path.cwd()}'

    @property
    @memoize
    def home(self) -> str:
        return f'{pathlib.Path.home()}'

    @property
    def path_base(self) -> str:
        return f'.rosdev/{self.architecture}'

    @property
    def path(self) -> str:
        return f'{self.path_base}/{self.build_num or self.release}'

    @property
    def cache_path_base(self) -> str:
        return f'{self.home}/{self.path_base}'

    @property
    def cache_path(self) -> str:
        return f'{self.home}/{self.path}'

    @property
    def install_path_base(self) -> str:
        return f'{self.cwd}/{self.path_base}'

    @property
    def install_path(self) -> str:
        return f'{self.cwd}/{self.path}'

    def __await__(self) -> Generator[_Install, None, None]:
        async def __await___inner() -> None:
            if self.build_num is None:
                return await self._install_from_container()
            else:
                return await self._install_from_osrf_build_farm()

        return __await___inner().__await__()

    @property
    def ros_distro(self) -> str:
        if (self.build_num is not None) or (self.release == 'latest'):
            return 'crystal'

        return self.release

    async def _install_from_container(self) -> None:
        log.info(f'Installing from docker image {self.release} to {self.install_path}')
        await container(
            architecture=self.architecture,
            build_num=self.build_num,
            release=self.release,
            ports=frozenset(),
            interactive=False,
            command=f'cp -r /opt/ros/{self.ros_distro} {self.install_path}',
        )

    async def _install_from_osrf_build_farm(self) -> None:
        log.info("Installing from OSRF build farm")
        if os.path.exists(self.cache_path):
            log.info(f'Found cached artifacts at {self.cache_path}')
        else:
            with TemporaryDirectory() as temp_dir:
                artifacts_path = f'{temp_dir}/artifacts.tar.bz2'
                temp_path = f'{temp_dir}/{self.path}'

                log.info('Downloading build artifacts')
                await exec(
                    f'wget https://ci.ros2.org/view/packaging/job/'
                    f'packaging_{get_operating_system(self.architecture)}/{self.build_num}'
                    f'/artifact/ws/ros2-package-linux-{get_machine(self.architecture)}.tar.bz2 '
                    f'-O {artifacts_path}'
                )

                log.info(f'Extracting build artifacts')
                os.makedirs(temp_path, exist_ok=True)
                await exec(f'tar -xf {artifacts_path} -C {temp_path} --strip-components 1')

                log.info(f'Caching artifacts to {self.cache_path}')
                await get_event_loop().run_in_executor(
                    None, partial(shutil.move, temp_path, self.cache_path))

        if os.path.exists(self.install_path):
            log.info(f'Removing previous install at {self.install_path}')
            await get_event_loop().run_in_executor(
                None, partial(shutil.rmtree, self.install_path, ignore_errors=True))

        log.info(f'Installing artifacts to {self.install_path}')
        os.makedirs(self.install_path_base, exist_ok=True)
        await get_event_loop().run_in_executor(
            None, partial(shutil.copytree, self.cache_path, self.install_path))


@memoize
async def install(
        *,
        architecture: str,
        build_num: Optional[str],
        release: str,
) -> None:
    await _Install(
        architecture=architecture,
        build_num=build_num,
        release=release,
    )
