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
    async def resolve_options(cls, options: Options) -> Options:
        rosdev_container_path = options.resolve_path(options.rosdev_container_path)
        rosdev_universal_path = options.resolve_path(options.rosdev_universal_path)
        rosdev_workspace_path = options.resolve_path(options.rosdev_workspace_path)

        return replace(
            options,
            rosdev_container_path=rosdev_container_path,
            rosdev_universal_path=rosdev_universal_path,
            rosdev_workspace_path=rosdev_workspace_path,
        )

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        # TODO py38 debug string
        log.debug(f'rosdev_container_path: {options.rosdev_container_path}')
        log.debug(f'rosdev_universal_path: {options.rosdev_universal_path}')
        log.debug(f'rosdev_workspace_path: {options.rosdev_workspace_path}')

        assert (
            '.rosdev' == options.rosdev_container_path.parts[-1]
        ), 'rosdev_container_path must end with .rosdev directory'
        assert (
            '.rosdev' == options.rosdev_universal_path.parts[-1]
        ), 'rosdev_universal_path must end with .rosdev directory'
        assert (
            '.rosdev' == options.rosdev_workspace_path.parts[-1]
        ), 'rosdev_workspace_path must end with .rosdev directory'

    @classmethod
    async def main(cls, options: Options) -> None:
        await gather(
            exec(f'mkdir -p {options.rosdev_universal_path}'),
            exec(f'mkdir -p {options.rosdev_workspace_path}'),
        )
