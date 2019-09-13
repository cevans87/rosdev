from asyncio import get_event_loop
from dataclasses import dataclass, field, replace
from jenkins import Jenkins
from logging import getLogger
from pathlib import Path
from typing import Tuple, Type

from rosdev.gen.architecture import GenArchitecture
from rosdev.gen.container import GenContainer
from rosdev.gen.release import GenRelease
from rosdev.gen.universal import GenUniversal
from rosdev.gen.workspace import GenWorkspace
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBuildNum(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenArchitecture,
        GenContainer,
        GenRelease,
        GenUniversal,
        GenWorkspace,
    ))

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        build_num = options.build_num
        if build_num is None:
            if options.release != 'latest':
                build_num = {
                    'amd64': {
                        'kinetic': 0,
                        'melodic': 0,
                        'dashing': 1482,
                        'crystal': 1289,
                    },
                    'arm32v7': {
                        'kinetic': 0,
                        'melodic': 0,
                        'crystal': 1,  # Note that this is just the first successful build.
                        'dashing': 16,
                    },
                    'arm64v8': {
                        'kinetic': 0,
                        'melodic': 0,
                        'dashing': 825,
                        'crystal': 651,
                    },
                }[options.architecture][options.release]
        if (build_num is None) and (not options.pull_build):
            for path in reversed(sorted(options.build_num_universal_path.glob('*'))):
                if Path(path, 'src').is_dir() and Path(path, 'install').is_dir():
                    build_num = int(path.name)
                    break

        if build_num is None:
            def get_build_num() -> int:
                return Jenkins('https://ci.ros2.org').get_job_info(
                    name=f'packaging_{options.operating_system}',
                    depth=1,
                    fetch_all_builds=False,
                )['lastSuccessfulBuild']['number']

            build_num = await get_event_loop().run_in_executor(None, get_build_num)

        return replace(options, build_num=build_num)

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        # TODO py38 debug print
        log.debug(f'build_num: {options.build_num}')

        assert options.build_num is not None, 'build_num must not be None'
