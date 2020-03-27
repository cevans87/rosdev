from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.docker.pam_environment import GenDockerPamEnvironment
from rosdev.gen.host import GenHost
from rosdev.gen.idea.c_kdbx import GenIdeaCKdbx
from rosdev.gen.idea.c_pwd import GenIdeaCPwd
from rosdev.gen.idea.home import GenIdeaHome
from rosdev.gen.idea.ide_base import GenIdeaIdeBase
from rosdev.gen.idea.security_xml import GenIdeaSecurityXml
from rosdev.gen.idea.workspace import GenIdeaWorkspace
from rosdev.gen.idea.pycharm.deployment_xml import GenIdeaPycharmDeploymentXml
from rosdev.gen.idea.pycharm.iml import GenIdeaPycharmIml
from rosdev.gen.idea.pycharm.jdk_table_xml import GenIdeaPycharmJdkTableXml
from rosdev.gen.idea.pycharm.misc_xml import GenIdeaPycharmMiscXml
from rosdev.gen.idea.pycharm.modules_xml import GenIdeaPycharmModulesXml
from rosdev.gen.idea.pycharm.webservers_xml import GenIdeaPycharmWebserversXml
from rosdev.gen.rosdep.install import GenRosdepInstall
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.path import Path


log = getLogger(__name__)


@dataclass(frozen=True)
class Pycharm(Handler):

    @classmethod
    @memoize
    async def get_path(cls, options: Options) -> Path:
        path = Path(await GenHost.execute_and_get_line(command='which charm', options=options))

        log.debug(f'{cls.__name__} {path = }')

        return path

    @classmethod
    async def main(cls, options: Options) -> None:
        log.info(f'Starting {await GenIdeaIdeBase.get_name(options)}.')
        await GenHost.execute_shell(
            command=(
                f'nohup {await cls.get_path(options)} {Path.workspace()}'
                f' < /dev/null > /dev/null 2>&1 &'
            ),
            options=options,
        )
