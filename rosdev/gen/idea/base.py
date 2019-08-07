from dataclasses import dataclass, field, replace
from logging import getLogger
from pathlib import Path
import sys
from typing import Tuple, Type

from rosdev.gen.base import GenBase
from rosdev.gen.idea.ide.name import GenIdeaIdeName
from rosdev.gen.idea.keepass import GenIdeaKeepass
from rosdev.gen.idea.security_xml import GenIdeaSecurityXml
from rosdev.gen.idea.universal import GenIdeaUniversal
from rosdev.gen.idea.uuid import GenIdeaUuid
from rosdev.gen.idea.workspace_xml import GenIdeaWorkspaceXml
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaBase(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenBase,
        GenIdeaIdeName,
        GenIdeaKeepass,
        GenIdeaSecurityXml,
        GenIdeaUniversal,
        GenIdeaUuid,
        GenIdeaWorkspaceXml,
    ))

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        idea_universal_path = options.idea_universal_path
        if idea_universal_path is None:
            if sys.platform == 'darwin':
                search_path = Path(Path.home(), 'Library', 'Preferences')
            else:
                search_path = Path.home()
            ide_paths = sorted(search_path.glob(f'.{options.idea_ide_name}*'))
            assert ide_paths, (
                f'Could not find any {options.idea_ide_name} settings directories in {search_path}'
            )
            idea_universal_path = ide_paths[-1]
        idea_universal_path = options.resolve_path(idea_universal_path)

        return replace(options, idea_universal_path=idea_universal_path)

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        # FIXME py38 debug print
        log.debug(f'idea_universal_path: {options.idea_universal_path}')
        log.debug(f'idea_workspace_path: {options.idea_workspace_path}')

        assert options.idea_universal_path is not None, 'idea_universal_path cannot be None'
        assert options.idea_workspace_path is not None, 'idea_workspace_path cannot be None'
