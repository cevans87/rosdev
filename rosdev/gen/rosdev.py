from asyncio import gather
from atools import memoize
from dataclasses import dataclass, field, replace
from logging import getLogger
from pathlib import Path
from typing import Tuple, Type

from rosdev.gen.base import GenBase
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.subprocess import exec


log = getLogger(__name__)


@memoize
@dataclass(frozen=True)
class GenRosdev(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenBase,
    ))

    @classmethod
    async def resolve_rosdev_container_path(cls, options: Options) -> Options:
        rosdev_container_path = options.resolve_container_path(options.rosdev_container_path)

        return replace(options, rosdev_container_path=rosdev_container_path)

    @classmethod
    async def resolve_rosdev_universal_path(cls, options: Options) -> Options:
        rosdev_universal_path = options.resolve_universal_path(options.rosdev_universal_path)

        return replace(options, rosdev_universal_path=rosdev_universal_path)

    @classmethod
    async def resolve_rosdev_workspace_path(cls, options: Options) -> Options:
        rosdev_workspace_path = options.rosdev_workspace_path

        if rosdev_workspace_path is None:
            for path in [Path.cwd(), *Path.cwd().parents]:
                path = Path(path, '.rosdev')
                if path.is_dir():
                    rosdev_workspace_path = path
                    break
            else:
                rosdev_workspace_path = Path(Path.cwd(), '.rosdev')

        rosdev_workspace_path = options.resolve_workspace_path(rosdev_workspace_path)

        return replace(options, rosdev_workspace_path=rosdev_workspace_path)

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        options = await cls.resolve_rosdev_container_path(options)
        options = await cls.resolve_rosdev_universal_path(options)
        options = await cls.resolve_rosdev_workspace_path(options)

        return options

    @classmethod
    async def main(cls, options: Options) -> None:
        # TODO validate options
        await gather(
            exec(f'mkdir -p {options.rosdev_universal_path}'),
            exec(f'mkdir -p {options.rosdev_workspace_path}'),
        )
