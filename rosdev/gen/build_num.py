import ast
from dataclasses import dataclass, field, replace
from logging import getLogger
from typing import Tuple, Type

from rosdev.gen.architecture import GenArchitecture
from rosdev.gen.container import GenContainer
from rosdev.gen.release import GenRelease
from rosdev.gen.universal import GenUniversal
from rosdev.gen.workspace import GenWorkspace
from rosdev.util.build_farm import get_build_num
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
            try:
                build_num = ast.literal_eval(options.read_text(path=options.build_num_universal_path))
            except FileNotFoundError:
                pass

        if build_num is None:
            build_num = await get_build_num(
                architecture=options.architecture, release=options.release
            )

        return replace(options, build_num=build_num)

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        # TODO py38 debug print
        log.debug(f'build_num: {options.build_num}')

        assert options.build_num is not None, 'build_num must not be None'

    @classmethod
    async def main(cls, options: Options) -> None:
        try:
            old_build_num = ast.literal_eval(
                options.read_text(path=options.build_num_universal_path)
            )
        except (FileNotFoundError, ValueError):
            old_build_num = None

        if (old_build_num is None) or (old_build_num < options.build_num):
            options.write_text(
                path=options.build_num_universal_path,
                text=f'{options.build_num}'
            )
