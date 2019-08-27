from dataclasses import dataclass, field
from logging import getLogger
from typing import Tuple, Type

from rosdev.gen.base import GenBase
from rosdev.gen.idea.ide.name import GenIdeaIdeName
from rosdev.gen.idea.keepass import GenIdeaKeepass
from rosdev.gen.idea.security_xml import GenIdeaSecurityXml
from rosdev.gen.idea.universal import GenIdeaUniversal
from rosdev.gen.idea.uuid import GenIdeaUuid
from rosdev.gen.idea.workspace_xml import GenIdeaWorkspaceXml
from rosdev.util.handler import Handler


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
