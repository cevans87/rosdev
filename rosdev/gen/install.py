from atools import memoize
from dataclasses import dataclass
from frozendict import frozendict
from logging import getLogger
import os
from tempfile import TemporaryDirectory
from typing import Mapping

from rosdev.gen.rosdev.config import Config as RosdevConfig
# from rosdev.gen.docker.container import Container
from rosdev.util.handler import Handler
from rosdev.util.lookup import get_build_num, get_machine, get_operating_system
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
        return f'{RosdevConfig(self.options).global_path}/install'

    @property
    def global_path(self) -> str:
        return f'{self.global_path_base}/' \
            f'{self.options.architecture}_{self.options.build_num or self.options.release}'

    @property
    def local_path_base(self) -> str:
        return RosdevConfig(self.options).local_path

    @property
    def local_path(self) -> str:
        return f'{self.local_path_base}/install'

    # TODO find a better place for this
    @property
    def ros_distro(self) -> str:
        if (self.options.build_num is not None) or (self.options.release == 'latest'):
            return 'crystal'

        return self.options.release

    @property
    def volumes(self) -> Mapping[str, str]:
        if self.options.global_setup is None:
            return self.options.volumes

        return frozendict({
            **self.options.volumes,
            self.local_path: self.container_path,
        })

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

        if self.options.build_num is not None or self.options.release != 'latest':
            await self._create_global_install_from_osrf_build_farm()
        else:
            raise NotImplemented()
            # await self._create_global_install_from_container()

        await exec(f'chmod -R -w {self.global_path}')

        log.info(f'Global install at {self.global_path}')

    # async def _create_global_install_from_container(self) -> None:
    #     log.info(
    #         f'Installing from docker image {self.options.release} to {self.global_install_path}')
    #     await exec(f'mkdir -p {self.global_install_path}')
    #     await Container(
    #         self.options(
    #             global_setup=None,
    #             command=f'cp -r /opt/ros/{self.ros_distro} {self.global_install_path}',
    #             interactive=False,
    #         )
    #     )

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

            log.info(f'Caching install at {self.global_path}')
            await exec(f'mkdir -p {self.global_path_base}')
            await exec(f'mv {staging_path} {self.global_path}')

    async def _create_local_install(self) -> None:
        log.info(f'Linking install at {self.local_path}')

        await exec(f'mkdir -p {self.local_path_base}', err_ok=True)
        await exec(f'rm {self.local_path}', err_ok=True)
        await exec(f'ln -s {self.global_path} {self.local_path}')

        log.info(f'Linked install at {self.local_path}')
