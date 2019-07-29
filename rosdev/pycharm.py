from dataclasses import dataclass, field
from logging import getLogger
from typing import Tuple, Type

from rosdev.gen.idea.security_xml import GenIdeaSecurityXml
from rosdev.gen.idea.webservers_xml import GenIdeaWebserversXml
from rosdev.gen.idea.workspace_xml import GenIdeaWorkspaceXml
from rosdev.gen.idea.deployment_xml import GenIdeaDeploymentXml
from rosdev.gen.idea.ide import GenIdeaIde
from rosdev.gen.docker.ssh.start import GenDockerSshStart
from rosdev.gen.idea.keepass import GenIdeaKeepass
from rosdev.gen.pam_environment import GenPamEnvironment
from rosdev.util.handler import Handler


log = getLogger(__name__)


@dataclass(frozen=True)
class Pycharm(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenDockerSshStart,
        GenIdeaDeploymentXml,
        GenIdeaKeepass,
        GenIdeaSecurityXml,
        GenIdeaWebserversXml,
        GenIdeaWorkspaceXml,
        GenPamEnvironment,
    ))
    post_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenIdeaIde,
    ))
