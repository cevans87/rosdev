from atools import memoize
from dataclasses import dataclass
import getpass
from logging import getLogger
from lxml import etree
from pathlib import Path
from textwrap import dedent
from typing import Tuple

from rosdev.gen.docker.container import GenDockerContainer
from rosdev.gen.docker.image import GenDockerImage
from rosdev.gen.docker.ssh import GenDockerSsh
from rosdev.gen.host import GenHost
from rosdev.gen.idea.home import GenIdeaHome
from rosdev.gen.idea.pycharm.webservers_xml import GenIdeaPycharmWebserversXml
from rosdev.gen.idea.workspace import GenIdeaWorkspace
from rosdev.gen.workspace import GenWorkspace
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.xml import get_root_element_from_path, merge_elements

log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaPycharmJdkTableXml(Handler):

    @classmethod
    @memoize
    async def get_path(cls, options: Options) -> Path:
        path = await GenIdeaHome.get_path(options) / 'options' / 'jdk.table.xml'
        
        log.debug(f'{cls.__name__} {path = }')
        
        return path

    @classmethod
    @memoize
    async def get_executable_path(cls, options: Options) -> Path:
        ros_python_version = (
            await GenDockerContainer.get_environment(options)
        )['ROS_PYTHON_VERSION']

        python_executable = Path(
            await GenDockerImage.execute_and_get_line(
                command=f'which python{ros_python_version}',
                options=options,
            )
        )
        
        log.debug(f'{python_executable = }')
        
        return python_executable

    @classmethod
    @memoize
    async def get_version(cls, options: Options) -> str:
        python_version = await GenDockerImage.execute_and_get_line(
            command=f'{await cls.get_executable_path(options)} --version',
            options=options,
        )
        
        log.debug(f'{python_version = }')
        
        return python_version

    @classmethod
    @memoize
    async def get_remote_paths(cls, options: Options) -> Tuple[Path, ...]:
        remote_paths = (
            await GenDockerContainer.get_environment(options)
        )['PYTHONPATH']
        
        remote_paths = remote_paths.split(':')
        remote_paths = [Path(remote_path) for remote_path in remote_paths]
        remote_paths = tuple([
            remote_path for remote_path in remote_paths
            if await GenWorkspace.get_path(options) in remote_path.parents
        ])

        log.debug(f'{remote_paths = }')
        
        return remote_paths

    @classmethod
    @memoize
    async def get_remote_address(cls, options) -> str:
        remote_address = (
                f'{getpass.getuser()}@localhost:{await GenDockerSsh.get_port(options)}'
        )
        
        log.debug(f'{cls.__name__} {remote_address = }')
        
        return remote_address

    @classmethod
    @memoize
    async def get_uri(cls, options: Options) -> str:
        uri = f'{await cls.get_remote_address(options)}{await cls.get_executable_path(options)}'
        
        log.debug(f'{cls.__name__} {uri = }')
        
        return uri

    @classmethod
    @memoize
    async def get_sftp_uri(cls, options: Options) -> str:
        sftp_uri = f'sftp://{await cls.get_uri(options)}'
        
        log.debug(f'{cls.__name__} {sftp_uri = }')
        
        return sftp_uri

    @classmethod
    @memoize
    async def get_ssh_uri(cls, options: Options) -> str:
        ssh_uri = f'ssh://{await cls.get_uri(options)}'
        
        log.debug(f'{cls.__name__} {ssh_uri = }')
        
        return ssh_uri

    @classmethod
    async def get_text(cls, options: Options) -> str:
        from_element = etree.fromstring(
            parser=etree.XMLParser(remove_blank_text=True),
            text=dedent(f'''
                <application>
                  <component name="ProjectJdkTable">
                    <jdk version="2">
                      <name value="{await GenIdeaPycharmWebserversXml.get_name(options)}" />
                      <type value="Python SDK" />
                      <version value="{await cls.get_version(options)}" />
                      <homePath value="{await cls.get_sftp_uri(options)}" />
                      <roots>
                        <classPath>
                          <root type="composite">
                            {
                            """
                            """.join([
                                f'<root url="file://{remote_path}" type="simple" />'
                                for remote_path in await cls.get_remote_paths(options)
                            ])
                            }
                          </root>
                        </classPath>
                        <sourcePath>
                          <root type="composite" />
                        </sourcePath>
                      </roots>
                      <additional
                          WEB_SERVER_CONFIG_ID="{await GenIdeaWorkspace.get_uuid(options)}" 
                          WEB_SERVER_CONFIG_NAME="{await cls.get_remote_address(options)}" 
                          WEB_SERVER_CREDENTIALS_ID="{await cls.get_sftp_uri(options)}" 
                          INTERPRETER_PATH="{await cls.get_executable_path(options)}"
                          HELPERS_PATH="$USER_HOME$/.pycharm_helpers"
                          INITIALIZED="false" 
                          VALID="true"
                          RUN_AS_ROOT_VIA_SUDO="false"
                          SKELETONS_PATH=""
                          VERSION="">
                        {
                        """
                        """.join([
                            f'<PATHS_ADDED_BY_USER_ROOT PATH_ADDED_BY_USER='
                            f'"file://{remote_path}" />'
                            for remote_path in await cls.get_remote_paths(options)
                        ])
                        }
                        <PathMappingSettings>
                          <option name="pathMappings">
                            <list>
                              {
                              """
                              """.join([
                                 f'<mapping'
                                 f' local-root="{remote_path}"'
                                 f' remote-root="{remote_path}" />'
                                 for remote_path in await cls.get_remote_paths(options)
                              ])
                              }
                            </list>
                          </option>
                        </PathMappingSettings>
                      </additional>
                    </jdk>
                  </component>
                </application>
            ''').strip()
        )
        into_element = get_root_element_from_path(await cls.get_path(options))
        element = merge_elements(from_element=from_element, into_element=into_element)
        text = etree.tostring(element, pretty_print=True, encoding=str)
        
        log.debug(f'{cls.__name__} {text = }')

        return text

    @classmethod
    async def main(cls, options: Options) -> None:
        GenHost.write_text(
            data=await cls.get_text(options),
            options=options,
            path=await cls.get_path(options),
        )
