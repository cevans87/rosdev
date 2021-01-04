from __future__ import annotations
from dataclasses import dataclass
from functools import wraps
from inspect import isabstract, iscoroutinefunction
import logging
import sys
from typing import Any, Callable

from rosdev.options import Options
from rosdev.third_party.atools import memoize, memoize_db


@dataclass(frozen=True)
class Node:
    options: Options

    class Exception(Exception):
        pass

    class Abort(Exception):
        def __str__(self) -> str:
            return f'Abort: {super().__str__()}'
    
    def __getattribute__(self, item) -> Any:
        if (
                (getattr(self, 'options').log_level != Options.LogLevel.DEBUG)
                or item.startswith('get_')
        ):
            return object.__getattribute__(self, item)

    def __hash__(self) -> int:
        return hash((type(self), self.options))

    def __repr__(self) -> str:
        return type(self).__name__

    def __str__(self) -> str:
        return repr(self)

    @property
    @memoize
    def log(self) -> logging.Logger:
        log = logging.getLogger(f'{self.__module__}')
        log.setLevel(self.options.log_level)
        log.propagate = False

        handler = logging.StreamHandler(sys.stderr)
        handler.setLevel(self.options.log_level)
        if handler.level > logging.DEBUG:
            formatter = logging.Formatter(f'%(message)s')
        else:
            formatter = logging.Formatter(f'%(levelname)s:%(name)s %(message)s')
        handler.setFormatter(formatter)
        log.addHandler(handler)

        return log

    @memoize
    async def cli_entrypoint(self) -> None:
        await self.run()

    @staticmethod
    def getter(fn: Callable[[Node], Any]) -> Any:
        if iscoroutinefunction(fn):
            async def decorated(node: Node) -> Any:
                result = await fn(node)

                node.log.debug(f'{fn.__name__.replace("get_", "", 1)} = {result}')

                return result
        else:
            def decorated(node: Node) -> Any:
                result = await fn(node)

                node.log.debug(f'{fn.__name__.replace("get_", "", 1)} = {result}')

                return result

        return wraps(fn)(decorated)

    memoize = memoize

    @staticmethod
    def memoize_getter(fn: Callable[[Node], Any], **kwargs) -> Any:
        return memoize(**kwargs)(Node.getter(fn))

    memoize_db = memoize_db

    @staticmethod
    def memoize_db_getter(fn: Callable[[Node], Any], **kwargs) -> Any:
        return memoize_db(**kwargs)(Node.getter(fn))
    
    @memoize
    async def run(self) -> None:
        if isabstract(self):
            return

        if hasattr(self, 'main'):
            self.log.debug(f'Starting {type(self).__name__}.main')
            await self.main()
            self.log.debug(f'Finished {type(self).__name__}.main')
