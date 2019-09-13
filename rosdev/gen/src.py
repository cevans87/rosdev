from asyncio import get_event_loop
from dataclasses import dataclass, field
from jenkins import Jenkins
from logging import getLogger
import re
from tempfile import TemporaryDirectory
from typing import List, Tuple, Type

from rosdev.gen.base import GenBase
from rosdev.util.handler import Handler
from rosdev.util.options import Options


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

        ros2_repos = await cls._get_ros2_repos(options)

        with TemporaryDirectory() as temp_dir:
            # TODO use pathlib operations to manipulate filesystem
            staging_path = f'{temp_dir}/src'
            ros2_repos_path = f'{temp_dir}/ros2.repos'
            with open(ros2_repos_path, 'w') as ros2_repos_f_out:
                ros2_repos_f_out.write(ros2_repos)

            log.info(f'Staging src at {staging_path}')
            await cls.exec_workspace(f'mkdir -p {staging_path}')
            await cls.exec_workspace(f'vcs import --input {ros2_repos_path} {staging_path}')

            log.info(f'Caching src at {options.src_universal_path}')
            await cls.exec_workspace(f'mkdir -p {options.src_universal_path.parent}')
            # FIXME this fails if the universal path already exists since we recursively made it
            #  read-only
            await cls.exec_workspace(f'mv {staging_path} {options.src_universal_path}')

        await cls.exec_workspace(f'chmod -R -w {options.src_universal_path}')

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

    @classmethod
    async def _get_ros2_repos(cls, options: Options) -> str:
        log.info("Finding src ros2.repos from OSRF build farm")

        def get_build_console_output() -> Tuple[str]:
            return tuple(
                Jenkins('https://ci.ros2.org').get_build_console_output(
                    name=f'packaging_{options.operating_system}',
                    number=options.build_num,
                ).splitlines()
            )

        remaining_lines = iter(
            await get_event_loop().run_in_executor(None, get_build_console_output)
        )
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

        return '\n'.join(ros2_repos_lines)
