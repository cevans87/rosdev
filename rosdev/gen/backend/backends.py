from asyncio import gather
from dataclasses import dataclass
from frozendict import frozendict
from logging import getLogger
from typing import Iterable, Mapping, Tuple, Type, TypeVar

from rosdev.gen.backend.base import GenBackendBase
from rosdev.gen.backend.builder import GenBackendBuilder
from rosdev.gen.backend.local import GenBackendLocal
from rosdev.gen.backend.runner import GenBackendRunner
from rosdev.util.handler import Handler
from rosdev.util.options import Options

log = getLogger(__name__)

_backends = frozenset({
    GenBackendBuilder,
    GenBackendLocal,
    GenBackendRunner,
})


T = TypeVar('T')


@dataclass(frozen=True)
class GenBackendBackends(Handler):

    @staticmethod
    async def execute_by_backend(
            command: str,
            options: Options,
            backends: Iterable[Type[GenBackendBase]] = _backends,
            err_ok=False,
    ) -> None:
        await gather(*[
            backend.execute(command=command, options=options, err_ok=err_ok)
            for backend in backends
        ])

    @staticmethod
    async def execute_and_get_lines_by_backend(
            command: str,
            options: Options,
            backends: Iterable[Type[GenBackendBase]] = _backends,
            err_ok=False,
    ) -> Mapping[Type[GenBackendBase], Tuple[str]]:
        lines_by_backend = {}

        async def execute_and_get_lines_by_backend_inner(
                backend: Type[GenBackendBase]
        ) -> None:
            lines_by_backend[backend] = await backend.execute_and_get_lines(
                command=command, options=options, err_ok=err_ok
            )
        await gather(*[execute_and_get_lines_by_backend_inner(backend) for backend in backends])
        
        return frozendict(lines_by_backend)

    @staticmethod
    async def execute_and_get_line_by_backend(
            command: str,
            options: Options,
            backends: Iterable[Type[GenBackendBase]] = _backends,
            err_ok=False,
    ) -> Mapping[Type[GenBackendBase], str]:
        line_by_backend = {}

        async def execute_and_get_line_by_backend_inner(
                backend: Type[GenBackendBase]
        ) -> None:
            line_by_backend[backend] = await backend.execute_and_get_line(
                command=command, options=options, err_ok=err_ok
            )
        await gather(*[execute_and_get_line_by_backend_inner(backend) for backend in backends])

        return frozendict(line_by_backend)
