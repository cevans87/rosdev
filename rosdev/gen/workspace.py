from dataclasses import dataclass, replace
from frozendict import frozendict
from logging import getLogger
from pathlib import Path
import re

from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenWorkspace(Handler):

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        workspace_path = options.workspace_path
        if workspace_path is None:
            for path in [Path.cwd(), *Path.cwd().parents]:
                if path == Path.home():
                    break
                elif Path(path, '.rosdev').is_dir():
                    workspace_path = path
                    break
        if workspace_path is None:
            workspace_path = Path.cwd()

        workspace_path = workspace_path.absolute()

        docker_container_volumes = dict(options.docker_container_volumes)
        docker_container_volumes[workspace_path] = workspace_path
        docker_container_volumes = frozendict(docker_container_volumes)

        workspace_hash = options.workspace_hash
        if workspace_hash is None:
            workspace_relative_path = workspace_path.relative_to(Path.home())
            workspace_hash = re.sub(r"[^\w.]", "_", str(workspace_relative_path))

        return replace(
            options,
            docker_container_volumes=docker_container_volumes,
            workspace_hash=workspace_hash,
            workspace_path=workspace_path,
        )

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        log.debug(f'{options.workspace_hash = }')
        log.debug(f'{options.workspace_path = }')

        assert options.workspace_hash is not None, f'Cannot be None: {options.workspace_hash = }'
        assert options.workspace_path is not None, f'Cannot be None: {options.workspace_path = }'
