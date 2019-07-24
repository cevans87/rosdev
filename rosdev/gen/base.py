from dataclasses import dataclass, replace
from logging import getLogger
from pathlib import Path

from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenBase(Handler):

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        base_workspace_path = options.base_workspace_path

        if base_workspace_path is None:
            for path in [Path.cwd(), *Path.cwd().parents]:
                if path == Path.home():
                    break
                elif Path(path, '.rosdev').is_dir():
                    base_workspace_path = path
                    break

        if base_workspace_path is None:
            base_workspace_path = Path.cwd()

        base_workspace_path = options.resolve_workspace_path(base_workspace_path)

        base_container_path = options.base_container_path

        if base_container_path is None:
            base_container_path = base_workspace_path

        base_container_path = options.resolve_container_path(base_container_path)

        base_universal_path = options.base_universal_path

        if base_universal_path is None:
            base_universal_path = Path.home()

        base_universal_path = options.resolve_universal_path(base_universal_path)

        return replace(
            options,
            base_container_path=base_container_path,
            base_workspace_path=base_workspace_path,
            base_universal_path=base_universal_path,
        )

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        # TODO py38 debug string
        log.debug(f'base_container_path: {options.base_container_path}')
        log.debug(f'base_universal_path: {options.base_universal_path}')
        log.debug(f'base_workspace_path: {options.base_workspace_path}')

        assert options.base_container_path is not None, 'base_container_path cannot be None'

        assert options.base_universal_path is not None, 'base_universal_path cannot be None'
        assert (
                (Path.home() == options.base_universal_path) or
                (Path.home() in options.base_universal_path.parents)
        ), f'base_universal_path must be home directory or a descendant of home directory'
        assert options.base_universal_path.is_dir(), 'base_universal_path must exist'

        assert options.base_workspace_path is not None, 'base_workspace_path cannot be None'
        assert (
            (Path.home() in options.base_workspace_path.parents)
        ), f'base_workspace_path must be a descendant of home directory'
        assert options.base_workspace_path.is_dir(), 'base_workspace_path must exist'
