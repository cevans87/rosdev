from dataclasses import dataclass, replace
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

        workspace_hash = options.workspace_hash
        if workspace_hash is None:
            workspace_relative_path = workspace_path.relative_to(Path.home())
            workspace_hash = re.sub(r"[^\w.]", "_", str(workspace_relative_path))

        return replace(
            options,
            workspace_hash=workspace_hash,
            workspace_path=workspace_path,
        )

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        # TODO py38 debug string
        log.debug(f'workspace_hash: {options.workspace_hash}')
        log.debug(f'workspace_path: {options.workspace_path}')

        assert options.workspace_hash is not None, 'workspace_hash cannot be None'
        assert options.workspace_path is not None, 'workspace_path cannot be None'
