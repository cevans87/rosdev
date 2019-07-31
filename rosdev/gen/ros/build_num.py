import ast
from dataclasses import dataclass, field, replace
from logging import getLogger
from pathlib import Path
from typing import Tuple, Type

from rosdev.gen.architecture import GenArchitecture
from rosdev.util.build_farm import get_ros_build_num
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.subprocess import exec


log = getLogger(__name__)


@dataclass(frozen=True)
class GenRosBuildNum(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenArchitecture,
    ))

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        ros_build_num_universal_path: Path = options.resolve_path(
            options.ros_build_num_universal_path
        )

        ros_build_num = options.ros_build_num
        if (ros_build_num is None) and (options.ros_release is not None):
            ros_build_num = {
                'amd64': {
                    'dashing': 1482,
                    'crystal': 1289,
                },
                'arm32v7': {
                    'dashing': 16,
                },
                'arm64v8': {
                    'dashing': 825,
                    'crystal': 651,
                },
            }.get(options.architecture, {}).get(options.ros_release)
        if (
                (ros_build_num is None) and
                (not options.pull_ros_install) and
                (not options.pull_ros_src)
        ):
            try:
                with open(Path(ros_build_num_universal_path), 'r') as f_in:
                    ros_build_num = ast.literal_eval(f_in.read())
            except FileNotFoundError:
                pass

        if ros_build_num is None:
            ros_build_num = await get_ros_build_num(
                architecture=options.architecture, ros_release=options.ros_release
            )

        return replace(
            options,
            ros_build_num=ros_build_num,
            ros_build_num_universal_path=ros_build_num_universal_path,
        )

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        # TODO py38 debug print
        log.debug(f'ros_build_num: {options.ros_build_num}')

        assert options.ros_build_num is not None, 'build_num must not be None'

    @classmethod
    async def main(cls, options: Options) -> None:
        try:
            with open(Path(options.ros_build_num_universal_path), 'r') as f_in:
                old_ros_build_num = int(ast.literal_eval(f_in.read()))
        except (FileNotFoundError, ValueError):
            old_ros_build_num = None

        if (old_ros_build_num is None) or (old_ros_build_num < options.ros_build_num):
            await exec(f'mkdir -p {options.ros_build_num_universal_path.parent}')
            with open(Path(options.ros_build_num_universal_path), 'w') as f_out:
                f_out.write(f'{options.ros_build_num}')
