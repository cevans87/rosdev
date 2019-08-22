from __future__ import annotations
from asyncio import gather
from atools import memoize
from dataclasses import dataclass, field, replace
from logging import getLogger
from pathlib import Path
from typing import Tuple, Type

from rosdev.util.options import Options
from rosdev.util.subprocess import get_exec_lines


log = getLogger(__name__)


@dataclass(frozen=True)
class Handler:
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=tuple())
    post_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=tuple())

    @classmethod
    async def run(cls, options: Options) -> None:
        options = replace(options, stage='resolve_options')
        options = await cls.__resolve_all_options(options)

        options = replace(options, stage='validate_options')
        if options.run_validate_options:
            await cls.__validate_all_options(options)
        else:
            log.warning('Skipping validate options')

        options = replace(options, stage='main')
        if options.run_main:
            await cls.__main_all(options)
        else:
            log.warning('Skipping main')

    @classmethod
    async def __pre_resolve_options(cls, options: Options) -> Options:
        for pre_dependency in cls.pre_dependencies:
            options = await pre_dependency.__resolve_all_options(options)

        return options

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        return options

    @classmethod
    async def __post_resolve_options(cls, options: Options) -> Options:
        for post_dependency in cls.post_dependencies:
            options = await post_dependency.__resolve_all_options(options)

        return options

    @classmethod
    async def __resolve_all_options(cls, options: Options) -> Options:
        options = await cls.__pre_resolve_options(options)
        options = await cls.resolve_options(options)
        options = await cls.__post_resolve_options(options)

        return options

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        pass

    @classmethod
    @memoize
    async def __validate_all_options(cls, options: Options) -> None:
        await gather(
            *[
                pre_dependency.__validate_all_options(options)
                for pre_dependency in cls.pre_dependencies
            ],
            cls.validate_options(options),
            *[
                post_dependency.__validate_all_options(options)
                for post_dependency in cls.post_dependencies
            ]
        )

    @classmethod
    async def __pre_main(cls, options: Options) -> None:
        await gather(*[
            pre_dependency.__main_all(options) for pre_dependency in cls.pre_dependencies
        ])

    @classmethod
    async def main(cls, options: Options) -> None:
        pass

    @classmethod
    async def __post_main(cls, options: Options) -> None:
        await gather(*[
            post_dependency.__main_all(options) for post_dependency in cls.post_dependencies
        ])

    @classmethod
    @memoize
    async def __main_all(cls, options: Options) -> None:
        await cls.__pre_main(options)
        # TODO debug print main's dockstring
        log.debug(f'Starting {cls.__name__}.main: {cls.main.__doc__ or "description unavailable"}')
        await cls.main(options)
        log.debug(f'Finished {cls.__name__}.main')
        await cls.__post_main(options)

    @classmethod
    async def exec_container(cls, options: Options, cmd: str, err_ok: bool = False) -> Tuple[str]:
        return await get_exec_lines(
            command=f'docker exec {options.docker_container_name} {cmd}',
            err_ok=err_ok
        )

    # noinspection PyUnusedLocal
    @classmethod
    async def exec_workspace(cls, options: Options, cmd: str, err_ok: bool = False) -> Tuple[str]:
        return await get_exec_lines(
            command=cmd,
            err_ok=err_ok
        )

    # noinspection PyUnusedLocal
    @classmethod
    def read(cls, *, options: Options, path: Path) -> str:
        with path.open('r') as f_in:
            return f_in.read()

    @classmethod
    def write(cls, *, options: Options, path: Path, text: str) -> None:
        assert options.stage == 'main', 'Cannot write files outside of main'

        if not options.dry_run:
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open('w') as f_out:
                f_out.write(text)
