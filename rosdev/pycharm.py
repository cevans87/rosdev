from asyncio import gather
from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.pycharm.deployment_xml import DeploymentXml
from rosdev.gen.pycharm.ide import Ide
from rosdev.gen.pycharm.keepass import Keepass
from rosdev.gen.pycharm.security_xml import SecurityXml
from rosdev.gen.pycharm.webservers_xml import WebserversXml
from rosdev.gen.pycharm.workspace_xml import WorkspaceXml
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
            DeploymentXml(self.options),
            Install(self.options),
            Keepass(self.options),
            SecurityXml(self.options),
            Src(self.options),
            WebserversXml(self.options),
            WorkspaceXml(self.options),
        )
        await Ide(self.options)
