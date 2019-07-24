from atools import memoize
from dataclasses import dataclass, field
from logging import getLogger
from typing import Tuple, Type

from rosdev.gen.clion.cpp_toolchains_xml import CppToolchainsXml
from rosdev.gen.clion.deployment_xml import DeploymentXml
from rosdev.gen.clion.ide import Ide
from rosdev.gen.clion.keepass import Keepass
from rosdev.gen.clion.security_xml import SecurityXml
from rosdev.gen.clion.toolchain import Toolchain
from rosdev.gen.clion.webservers_xml import WebserversXml
from rosdev.gen.clion.workspace_xml import WorkspaceXml
from rosdev.gen.install import Install
from rosdev.gen.src import Src
from rosdev.util.handler import Handler


log = getLogger(__name__)


@memoize
@dataclass(frozen=True)
class Clion(Handler):

    dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        CppToolchainsXml,
        DeploymentXml,
        Install,
        Keepass,
        SecurityXml,
        Src,
        Toolchain,
        WebserversXml,
        WorkspaceXml,
    ))

    @memoize
    async def _main(self) -> None:
        await Ide(self.options)
