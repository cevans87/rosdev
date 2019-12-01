from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.host import GenHost
from rosdev.gen.idea.clion.cpp_toolchains_xml import GenIdeaClionCppToolchainsXml
from rosdev.gen.idea.clion.deployment_xml import GenIdeaClionDeploymentXml
from rosdev.gen.idea.clion.iml import GenIdeaClionIml
from rosdev.gen.idea.clion.misc_xml import GenIdeaClionMiscXml
from rosdev.gen.idea.clion.modules_xml import GenIdeaClionModulesXml
from rosdev.gen.idea.clion.webservers_xml import GenIdeaClionWebserversXml
from rosdev.gen.idea.clion.workspace_xml import GenIdeaClionWorkspaceXml
from rosdev.gen.idea.c_kdbx import GenIdeaCKdbx
from rosdev.gen.idea.c_pwd import GenIdeaCPwd
from rosdev.gen.idea.home import GenIdeaHome
from rosdev.gen.idea.ide_base import GenIdeaIdeBase
from rosdev.gen.idea.security_xml import GenIdeaSecurityXml
from rosdev.gen.idea.workspace import GenIdeaWorkspace
from rosdev.gen.workspace import GenWorkspace
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.path import Path


log = getLogger(__name__)


@dataclass(frozen=True)
class Clion(Handler):

    @classmethod
    @memoize
    async def get_path(cls, options: Options) -> Path:
        path = Path(await GenHost.execute_and_get_line(command='which clion', options=options))

        log.debug(f'{cls.__name__} {path = }')

        return path

    @classmethod
    async def main(cls, options: Options) -> None:
        log.info(f'Starting {await GenIdeaIdeBase.get_name(options)}.')
        await GenHost.execute_shell(
            command=(
                f'nohup {await cls.get_path(options)} {await GenWorkspace.get_path(options)}'
                f'< /dev/null > /dev/null 2>&1 &'
            ),
            options=options,
        )
