from dataclasses import dataclass, field, replace
from logging import getLogger
from tempfile import TemporaryDirectory
from typing import Tuple, Type

from rosdev.gen.ros.build_num import GenRosBuildNum
from rosdev.util.build_farm import get_artifacts_url
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.subprocess import exec


log = getLogger(__name__)


@dataclass(frozen=True)
class GenRosInstall(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenRosBuildNum,
    ))

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        ros_install_container_path = options.resolve_path(
            options.ros_install_container_path
        )
        ros_install_universal_path = options.resolve_path(
            options.ros_install_universal_path
        )
        ros_install_workspace_path = options.resolve_path(
            options.ros_install_workspace_path
        )

        return replace(
            options,
            ros_install_container_path=ros_install_container_path,
            ros_install_universal_path=ros_install_universal_path,
            ros_install_workspace_path=ros_install_workspace_path,
        )

    @classmethod
    async def main(cls, options: Options) -> None:
        await cls._create_ros_install_universal(options)
        await cls._create_ros_install_workspace(options)

    @classmethod
    async def _create_ros_install_universal(cls, options: Options) -> None:
        if options.ros_install_universal_path.is_dir():
            log.info(f'Found install cached at {options.ros_install_universal_path}')
            return

        log.info("Finding artifacts url from OSRF build farm")
        artifacts_url = await get_artifacts_url(
            architecture=options.architecture,
            ros_build_num=options.ros_build_num,
            ros_release=options.ros_release,
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

            log.info(f'Caching install at {options.ros_install_universal_path}')
            await exec(f'mkdir -p {options.ros_install_universal_path.parent}')
            # FIXME this fails if the universal path already exists since we recursively made it
            #  read-only
            await exec(f'mv {staging_path} {options.ros_install_universal_path}')

        await exec(f'chmod -R -w {options.ros_install_universal_path}')

        log.info(f'Universal install at {options.ros_install_universal_path}')

    @classmethod
    async def _create_ros_install_workspace(cls, options: Options) -> None:
        log.info(f'Linking install at {options.ros_install_workspace_path}')

        await exec(f'mkdir -p {options.ros_install_workspace_path.parent}', err_ok=True)
        await exec(f'rm {options.ros_install_workspace_path}', err_ok=True)
        await exec(
            f'ln -s {options.ros_install_universal_path} {options.ros_install_workspace_path}'
        )

        log.info(f'Linked install at {options.ros_install_workspace_path}')
