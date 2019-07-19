from asyncio import gather
from atools import memoize
from dataclasses import dataclass, replace
from logging import getLogger

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

    @memoize
    async def _main(self) -> None:
        await gather(
            CppToolchainsXml(self.options),
            DeploymentXml(self.options),
            Install(self.options),
            Keepass(self.options),
            SecurityXml(self.options),
            Src(self.options),
            Toolchain(self.options),
            WebserversXml(self.options),
            WorkspaceXml(self.options),
        )
        await Ide(self.options)
