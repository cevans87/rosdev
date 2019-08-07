from dataclasses import dataclass, replace
from logging import getLogger
from pathlib import Path

from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenHome(Handler):

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        home_universal_path = options.home_universal_path
        if home_universal_path is None:
            home_universal_path = Path.home()

        home_container_path = options.home_container_path
        if home_container_path is None:
            home_container_path = home_universal_path

        return replace(
            options,
            home_container_path=home_container_path,
            home_universal_path=home_universal_path,
        )

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        # TODO py38 debug string
        log.debug(f'home_container_path: {options.home_container_path}')
        log.debug(f'home_universal_path: {options.home_universal_path}')

        assert options.home_container_path is not None, 'home_container_path cannot be None'

        assert (
            (Path.home() == options.home_universal_path) or
            (Path.home() in options.home_universal_path.parents)
        ), f'home_universal_path must be home directory or a descendant of home directory'
        assert options.home_universal_path.is_dir(), 'home_workspace_path must exist'
