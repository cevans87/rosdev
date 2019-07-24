from dataclasses import dataclass, field, replace
from frozendict import frozendict
from logging import getLogger
from tempfile import TemporaryDirectory
from typing import Tuple, Type

from rosdev.gen.ros.build.num import GenRosBuildNum
from rosdev.util.build_farm import get_ros2_repos
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.subprocess import exec


log = getLogger(__name__)


@dataclass(frozen=True)
class GenRosSrc(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenRosBuildNum,
    ))

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        ros_src_container_path = options.resolve_container_path(
            options.ros_src_container_path
        )
        ros_src_universal_path = options.resolve_universal_path(
            options.ros_src_universal_path
        )
        ros_src_workspace_path = options.resolve_workspace_path(
            options.ros_src_workspace_path
        )

        return replace(
            options,
            ros_src_container_path=ros_src_container_path,
            ros_src_universal_path=ros_src_universal_path,
            ros_src_workspace_path=ros_src_workspace_path,
        )

    @classmethod
    async def main(cls, options: Options) -> None:
        await cls._create_ros_src_universal(options)
        await cls._create_ros_src_workspace(options)

    @classmethod
    async def _create_ros_src_universal(cls, options: Options) -> None:
        if options.ros_src_universal_path.is_dir():
            log.info(f'Found src cached at {options.ros_src_universal_path}')
            return

        log.info("Finding src ros2.repos from OSRF build farm")
        ros2_repos = await get_ros2_repos(
            architecture=options.architecture,
            ros_build_num=options.ros_build_num,
            ros_release=options.ros_release,
        )
        with TemporaryDirectory() as temp_dir:
            staging_path = f'{temp_dir}/src'
            ros2_repos_path = f'{temp_dir}/ros2.repos'
            with open(ros2_repos_path, 'w') as ros2_repos_f_out:
                ros2_repos_f_out.write(ros2_repos)

            log.info(f'Staging src at {staging_path}')
            await exec(f'mkdir -p {staging_path}')
            await exec(f'vcs import --input {ros2_repos_path} {staging_path}')

            log.info(f'Caching src at {options.ros_src_universal_path}')
            await exec(f'mkdir -p {options.ros_src_universal_path.parent}')
            # FIXME this fails if the universal path already exists since we recursively made it
            #  read-only
            await exec(f'mv {staging_path} {options.ros_src_universal_path}')

        await exec(f'chmod -R -w {options.ros_src_universal_path}')

        log.info(f'Universal src at {options.ros_src_universal_path}')

    @classmethod
    async def _create_ros_src_workspace(cls, options: Options) -> None:
        log.info(f'Linking src at {options.ros_src_workspace_path}')

        await exec(f'mkdir -p {options.ros_src_workspace_path.parent}', err_ok=True)
        await exec(f'rm {options.ros_src_workspace_path}', err_ok=True)
        await exec(f'ln -s {options.ros_src_universal_path} {options.ros_src_workspace_path}')

        log.info(f'Linked src at {options.ros_src_workspace_path}')
