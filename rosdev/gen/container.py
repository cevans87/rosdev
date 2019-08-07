from dataclasses import dataclass, field, replace
from logging import getLogger
from typing import Tuple, Type

from rosdev.gen.workspace import GenWorkspace
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenContainer(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenWorkspace,
    ))

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        container_path = options.container_path

        if container_path is None:
            container_path = options.workspace_path

        container_path = options.resolve_path(container_path)

        return replace(options, container_path=container_path)

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        # TODO py38 debug string
        log.debug(f'container_path: {options.container_path}')

        assert options.container_path is not None, 'container_path cannot be None'
