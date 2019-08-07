from dataclasses import dataclass, field
from logging import getLogger
from typing import Tuple, Type

from rosdev.gen.idea.clion.cpp_toolchains_xml import GenIdeaClionCppToolchainsXml
from rosdev.gen.idea.clion.deployment_xml import GenIdeaClionDeploymentXml
from rosdev.gen.idea.clion.webservers_xml import GenIdeaClionWebserversXml
from rosdev.util.handler import Handler

log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaClionBase(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenIdeaClionCppToolchainsXml,
        GenIdeaClionDeploymentXml,
        GenIdeaClionWebserversXml,
    ))
