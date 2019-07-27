from __future__ import annotations
from asyncio import gather
from atools import memoize
from dataclasses import dataclass, field
from logging import getLogger
from typing import Tuple, Type

from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class Handler:
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=tuple())
    post_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=tuple())

    @classmethod
    async def run(cls, options: Options) -> None:
        options = await cls.__resolve_all_options(options)
        await cls.__validate_all_options(options)
        await cls.__main_all(options)

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
        await cls.main(options)
        await cls.__post_main(options)
