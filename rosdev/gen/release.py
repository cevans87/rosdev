from dataclasses import dataclass, field, replace
from logging import getLogger
from typing import Tuple, Type

from rosdev.gen.architecture import GenArchitecture
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenRelease(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenArchitecture,
    ))

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        release = options.release
        # FIXME this does not account for ROS1
        # FIXME some of these numbers are wrong and the list is incomplete
        if release == 'latest':
            if options.build_num is None:
                # FIXME programmatically find this.
                release = 'dashing'
            elif options.architecture == 'amd64':
                if 0 <= options.build_num < 1482:
                    release = 'crystal'
                elif 1482 <= options.build_num:
                    release = 'dashing'
            elif options.architecture == 'arm32v7':
                if 0 <= options.build_num < 16:
                    release = 'crystal'
                elif 16 <= options.build_num:
                    release = 'dashing'
            elif options.architecture == 'arm64v8':
                if 651 <= options.build_num < 825:
                    release = 'crystal'
                elif 825 <= options.build_num:
                    release = 'dashing'

        return replace(options, release=release)

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        # TODO py38 debug print
        log.debug(f'release: {options.release}')

        assert options.release is not None, 'release must not be None'
        assert options.release != 'latest', 'release must not be latest'
