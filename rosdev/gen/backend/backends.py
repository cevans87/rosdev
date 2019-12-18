from asyncio import gather
from dataclasses import dataclass
from frozendict import frozendict
from logging import getLogger
from typing import Iterable, Mapping, Tuple, Type

from rosdev.gen.backend.mixin_base import GenBackendMixinBase
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


@dataclass(frozen=True)
class GenBackendBackends(Handler):

    @staticmethod
    async def execute_by_backend(
            command: str,
            options: Options,
            backends: Iterable[Type[GenBackendMixinBase]] = _backends,
            err_ok=False,
    ) -> None:
        await gather(*[
            backend.get_ssh(options).execute(command=command, options=options, err_ok=err_ok)
            for backend in backends
        ])

    @staticmethod
    async def execute_and_get_lines_by_backend(
            command: str,
            options: Options,
            backends: Iterable[Type[GenBackendMixinBase]] = _backends,
            err_ok=False,
    ) -> Mapping[Type[GenBackendMixinBase], Tuple[str]]:
        lines_by_backend = {}

        async def execute_and_get_lines_by_backend_inner(
                backend: Type[GenBackendMixinBase]
        ) -> None:
            lines_by_backend[backend] = await backend.get_ssh(options).execute_and_get_lines(
                command=command, options=options, err_ok=err_ok
            )
        await gather(*[execute_and_get_lines_by_backend_inner(backend) for backend in backends])
        
        return frozendict(lines_by_backend)

    @staticmethod
    async def execute_and_get_line_by_backend(
            command: str,
            options: Options,
            backends: Iterable[Type[GenBackendMixinBase]] = _backends,
            err_ok=False,
    ) -> Mapping[Type[GenBackendMixinBase], str]:
        line_by_backend = {}

        async def execute_and_get_line_by_backend_inner(
                backend: Type[GenBackendMixinBase]
        ) -> None:
            line_by_backend[backend] = await backend.get_ssh(options).execute_and_get_line(
                command=command, options=options, err_ok=err_ok
            )
        await gather(*[execute_and_get_line_by_backend_inner(backend) for backend in backends])

        return frozendict(line_by_backend)
