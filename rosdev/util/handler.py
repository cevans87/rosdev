from __future__ import annotations
from asyncio import gather
from atools import memoize
from dataclasses import dataclass, field, replace
from logging import getLogger
from pathlib import Path
from typing import Tuple, Type

from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class Handler:
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=tuple())
    post_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=tuple())
    
    @classmethod
    async def run(cls, options: Options) -> None:
        options = replace(options, stage='resolve_options')
        options = await cls._resolve_all_options(options)

        options = replace(options, stage='validate_options')
        if options.run_validate_options:
            await cls._validate_all_options(options)
        else:
            log.warning('Skipping validate options')

        options = replace(options, stage='main')
        await cls._main_all(options)

    @classmethod
    async def _pre_resolve_options(cls, options: Options) -> Options:
        for pre_dependency in cls.pre_dependencies:
            options = await pre_dependency._resolve_all_options(options)

        return options

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        return options

    @classmethod
    async def _post_resolve_options(cls, options: Options) -> Options:
        for post_dependency in cls.post_dependencies:
            options = await post_dependency._resolve_all_options(options)

        return options

    @classmethod
    async def _resolve_all_options(cls, options: Options) -> Options:
        options = await cls._pre_resolve_options(options)
        options = await cls.resolve_options(options)
        options = await cls._post_resolve_options(options)

        return options

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        pass

    @classmethod
    @memoize
    async def _validate_all_options(cls, options: Options) -> None:
        await gather(
            *[
                pre_dependency._validate_all_options(options)
                for pre_dependency in cls.pre_dependencies
            ],
            cls.validate_options(options),
            *[
                post_dependency._validate_all_options(options)
                for post_dependency in cls.post_dependencies
            ]
        )

    @classmethod
    async def _pre_main(cls, options: Options) -> None:
        await gather(*[
            pre_dependency._main_all(options) for pre_dependency in cls.pre_dependencies
        ])

    @classmethod
    async def main(cls, options: Options) -> None:
        pass

    @classmethod
    async def _post_main(cls, options: Options) -> None:
        await gather(*[
            post_dependency._main_all(options) for post_dependency in cls.post_dependencies
        ])

    @classmethod
    @memoize
    async def _main_all(cls, options: Options) -> None:
        await cls._pre_main(options)
        # TODO debug print main's docstring
        log.debug(f'Starting {cls.__name__}.main: {cls.main.__doc__ or "description unavailable"}')
        if options.run_main:
            await cls.main(options)
        else:
            log.warning(f'Skipping {cls.__name__}.main')
        log.debug(f'Finished {cls.__name__}.main')
        await cls._post_main(options)
