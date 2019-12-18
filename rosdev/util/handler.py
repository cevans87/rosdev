from __future__ import annotations
from asyncio import gather, ensure_future
from atools import memoize
from dataclasses import dataclass
from importlib import import_module
from inspect import isabstract, isclass, iscoroutinefunction, Parameter, signature
from logging import getLogger

from rosdev.util.options import Options


log = getLogger(__name__)

_options_parameter = Parameter(
    name='options',
    kind=Parameter.POSITIONAL_OR_KEYWORD,
    annotation=Options,
)


@dataclass(frozen=True)
class Handler:

    @classmethod
    @memoize
    async def run(cls, options: Options) -> None:
        await cls._pre_main(options)

        if isabstract(cls):
            return

        if options.run_main and hasattr(cls, 'main'):
            log.debug(
                f'Starting {cls.__name__}.main: {cls.main.__doc__ or "description unavailable"}'
            )
            await cls.main(options)
            log.debug(f'Finished {cls.__name__}.main')

        await cls._post_main(options)

    @classmethod
    async def _pre_main(cls, options: Options) -> None:
        dependencies = []
        module = import_module(cls.__module__)
        for module_item in module.__dict__.values():
            if (
                    isclass(module_item) and
                    issubclass(module_item, Handler) and
                    (module_item is not cls) and
                    (module_item is not Handler)
            ):
                dependencies.append(module_item)
                
        log.debug(
            f'{cls.__name__} dependencies {[dependency.__name__ for dependency in dependencies]}'
        )

        await gather(*[dependency.run(options) for dependency in dependencies])

    @classmethod
    async def _post_main(cls, options: Options) -> None:
        # Kick off all the getters so that they might be ready earlier for data dependencies.
        getters = []
        for class_item_name in cls.__dict__.keys():
            if (
                    (
                            class_item_name.startswith('get_') or
                            class_item_name.startswith('set_')
                    ) and
                    iscoroutinefunction(class_item := getattr(cls, class_item_name)) and
                    (signature(class_item).parameters == {'options': _options_parameter}) and
                    hasattr(class_item, 'memoize')
            ):
                getters.append(class_item)
                
        getters_future = ensure_future(gather(*[getter(options) for getter in getters]))
        
        private_getters = []
        for class_item_name in cls.__dict__.keys():
            if (
                    (
                            class_item_name.startswith('_get_') or
                            class_item_name.startswith('_set_')
                    ) and
                    iscoroutinefunction(class_item := getattr(cls, class_item_name)) and
                    (signature(class_item).parameters == {'options': _options_parameter}) and
                    hasattr(class_item, 'memoize')
            ):
                getters.append(class_item)

        if private_getters:
            async def forget_private_getters() -> None:
                await getters_future
                for private_getter in private_getters:
                    private_getter.memoize.reset()

            ensure_future(forget_private_getters())
