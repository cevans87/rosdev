from dataclasses import dataclass, field
from logging import getLogger
from tempfile import TemporaryDirectory
from typing import Tuple, Type

from rosdev.gen.base import GenBase
from rosdev.util.build_farm import get_ros2_repos
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.subprocess import execute_command


log = getLogger(__name__)


@dataclass(frozen=True)
class GenSrc(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenBase,
    ))

    @classmethod
    async def main(cls, options: Options) -> None:
        if options.release in {'kinetic', 'melodic'}:
            log.info(f'Bypassing ROS 1 src download')
            return

        await cls._create_src_universal(options)
        await cls._create_src_workspace(options)

    @classmethod
    async def _create_src_universal(cls, options: Options) -> None:
        if options.src_universal_path.exists():
            log.info(f'Found src cached at {options.src_universal_path}')
            return

        log.info("Finding src ros2.repos from OSRF build farm")
        ros2_repos = await get_ros2_repos(
            architecture=options.architecture,
            build_num=options.build_num,
            release=options.release,
        )
        with TemporaryDirectory() as temp_dir:
            # TODO use pathlib operations to manipulate filesystem
            staging_path = f'{temp_dir}/src'
            ros2_repos_path = f'{temp_dir}/ros2.repos'
            with open(ros2_repos_path, 'w') as ros2_repos_f_out:
                ros2_repos_f_out.write(ros2_repos)

            log.info(f'Staging src at {staging_path}')
            await execute_command(f'mkdir -p {staging_path}')
            await execute_command(f'vcs import --input {ros2_repos_path} {staging_path}')

            log.info(f'Caching src at {options.src_universal_path}')
            await execute_command(f'mkdir -p {options.src_universal_path.parent}')
            # FIXME this fails if the universal path already exists since we recursively made it
            #  read-only
            await execute_command(f'mv {staging_path} {options.src_universal_path}')

        await execute_command(f'chmod -R -w {options.src_universal_path}')

        log.info(f'Universal src at {options.src_universal_path}')

    @classmethod
    async def _create_src_workspace(cls, options: Options) -> None:
        log.info(f'Linking src at {options.src_workspace_path}')

        options.src_workspace_path.parent.mkdir(parents=True, exist_ok=True)
        if options.src_workspace_path.exists():
            options.src_workspace_path.unlink()
        options.src_workspace_path.symlink_to(
            options.src_universal_path,
            target_is_directory=True,
        )

        log.info(f'Linked src at {options.src_workspace_path}')
