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
from rosdev.gen.rosdev.config import Config as RosdevConfig
from rosdev.gen.install import Install
from rosdev.gen.src import Src
from rosdev.util.handler import Handler


log = getLogger(__name__)


@memoize
@dataclass(frozen=True)
class Clion(Handler):

    @memoize
    async def _main(self) -> None:
        # FIXME try resolving the build_num and pass it as part of options that built this
        #  object. It's clunky having to do this for every top-level command.
        build_num = await RosdevConfig(self.options).get_build_num()
        options = replace(
            self.options,
            build_num=build_num,
        )
        await gather(
            CppToolchainsXml(options),
            DeploymentXml(options),
            Install(options),
            Keepass(options),
            SecurityXml(options),
            Src(options),
            Toolchain(options),
            WebserversXml(options),
            WorkspaceXml(options),
        )
        await Ide(options)
