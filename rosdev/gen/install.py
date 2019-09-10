#from asyncio import get_event_loop
#from atools import memoize
from dataclasses import dataclass, field
from logging import getLogger
from tempfile import TemporaryDirectory
from typing import Tuple, Type

from rosdev.gen.base import GenBase
from rosdev.util.build_farm import get_artifacts_url
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.subprocess import execute_command


log = getLogger(__name__)


@dataclass(frozen=True)
class GenInstall(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenBase,
    ))

    #@classmethod
    #@memoize
    #async def get_build_num(cls, architecture: str, release: str) -> int:
    #    if release != 'latest':
    #        return {
    #            'amd64': {
    #                'dashing': 1482,
    #                'crystal': 1289,
    #            },
    #            'arm32v7': {
    #                'dashing': 16,
    #            },
    #            'arm64v8': {
    #                'dashing': 825,
    #                'crystal': 651,
    #            },
    #        }[architecture][release]
    #    else:
    #        def get_build_num_inner() -> int:
    #            return self._external_jenkins.get_job_info(
    #                name=self.get_job_name(architecture=architecture),
    #                depth=1,
    #                fetch_all_builds=False,
    #            )['lastSuccessfulBuild']['number']

    #        return await get_event_loop().run_in_executor(None, get_build_num_inner)
    #@memoize
    #async def get_artifacts_url(
    #        cls,
    #        options: Options,
    #) -> str:
    #    if options.build_num is None:
    #        async with _JenkinsContext() as jenkins:
    #            build_num = await jenkins.get_build_num(
    #                architecture=architecture, release=release
    #            )

    #    return (
    #        f'https://ci.ros2.org/view/packaging/job/'
    #        f'packaging_{get_operating_system(architecture)}/{build_num}/artifact/'
    #        f'ws/ros2-package-linux-{get_machine(architecture)}.tar.bz2'
    #    )

    @classmethod
    async def main(cls, options: Options) -> None:
        if options.release in {'kinetic', 'melodic'}:
            log.info(f'Bypassing ROS 1 install download')
            return

        await cls._create_install_universal(options)
        await cls._create_install_workspace(options)

    @classmethod
    async def _create_install_universal(cls, options: Options) -> None:
        if options.install_universal_path.exists():
            log.info(f'Found install cached at {options.install_universal_path}')
            return

        log.info("Finding artifacts url from OSRF build farm")
        artifacts_url = await get_artifacts_url(
            architecture=options.architecture,
            build_num=options.build_num,
            release=options.release,
        )

        log.info('Installing from OSRF build farm')
        with TemporaryDirectory() as temp_dir:
            # TODO use pathlib operations to manipulate filesystem
            staging_path = f'{temp_dir}/install'
            artifacts_path = f'{temp_dir}/artifacts.tar.bz2'

            log.info(f'Downloading install artifacts to {artifacts_path}')
            await execute_command(f'wget {artifacts_url} -O {artifacts_path}')

            log.info(f'Staging install at {staging_path}')
            await execute_command(f'mkdir -p {staging_path}')
            await execute_command(
                f'tar -xf {artifacts_path} -C {staging_path} --strip-components 1'
            )

            log.info(f'Caching install at {options.install_universal_path}')
            await execute_command(f'mkdir -p {options.install_universal_path.parent}')
            # FIXME this fails if the universal path already exists since we recursively made it
            #  read-only
            await execute_command(f'mv {staging_path} {options.install_universal_path}')

        await execute_command(f'chmod -R -w {options.install_universal_path}')

        log.info(f'Universal install at {options.install_universal_path}')

    @classmethod
    async def _create_install_workspace(cls, options: Options) -> None:
        log.info(f'Linking install at {options.install_workspace_path}')

        options.install_workspace_path.parent.mkdir(parents=True, exist_ok=True)
        if options.install_workspace_path.exists():
            options.install_workspace_path.unlink()
        options.install_workspace_path.symlink_to(
            options.install_universal_path,
            target_is_directory=True,
        )

        log.info(f'Linked install at {options.install_workspace_path}')
