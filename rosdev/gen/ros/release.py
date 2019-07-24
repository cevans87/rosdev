from dataclasses import dataclass, field, replace
from logging import getLogger
from typing import Tuple, Type

from rosdev.gen.architecture import GenArchitecture
from rosdev.gen.ros.build.num import GenRosBuildNum
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenRosRelease(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenArchitecture,
        GenRosBuildNum,
    ))

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        ros_release = options.ros_release
        # FIXME this does not account for ROS1
        # FIXME some of these numbers are wrong and the list is incomplete
        if options.architecture == 'amd64':
            if 0 <= options.ros_build_num < 1482:
                ros_release = 'crystal'
            elif 1482 <= options.ros_build_num:
                ros_release = 'dashing'
        elif options.architecture == 'arm32v7':
            if 0 <= options.ros_build_num < 16:
                ros_release = 'crystal'
            elif 16 <= options.ros_build_num:
                ros_release = 'dashing'
        elif options.architecture == 'arm64v8':
            if 651 <= options.ros_build_num < 825:
                ros_release = 'crystal'
            elif 825 <= options.ros_build_num:
                ros_release = 'dashing'

        return replace(options, ros_release=ros_release)

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        # TODO py38 debug print
        log.debug(f'ros_release: {options.ros_release}')

        assert options.ros_release is not None, 'ros_release must not be None'
        assert options.ros_release != 'latest', 'ros_release must be an actual release'
