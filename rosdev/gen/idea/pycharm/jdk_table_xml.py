from atools import memoize
from dataclasses import dataclass, field
import getpass
from logging import getLogger
from lxml import etree
# noinspection PyProtectedMember
from lxml.etree import _Element
from pathlib import Path
from textwrap import dedent
from typing import FrozenSet, Tuple, Type

from rosdev.gen.docker.container import GenDockerContainer
from rosdev.gen.docker.image import GenDockerImage
from rosdev.gen.host import GenHost
from rosdev.gen.idea.ide.name import GenIdeaIdeName
from rosdev.gen.idea.universal import GenIdeaUniversal
from rosdev.gen.idea.uuid import GenIdeaUuid
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.xml import get_root_element_from_path, merge_elements

log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaPycharmJdkTableXml(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenDockerContainer,
        GenDockerImage,
        GenHost,
        GenIdeaIdeName,
        GenIdeaUniversal,
        GenIdeaUuid,
    ))

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        log.debug(f'{options.idea_pycharm_jdk_table_xml_path = }')

    @classmethod
    @memoize
    async def get_python_executable(cls, options: Options) -> str:
        ros_python_version = (
            await GenDockerContainer.get_environment(options)
        )['ROS_PYTHON_VERSION']

        python_executable = await GenDockerImage.execute_and_get_line(
            command=f'which python{ros_python_version}',
            options=options,
        )
        
        log.debug(f'{python_executable = }')
        
        return python_executable

    @classmethod
    @memoize
    async def get_python_version(cls, options: Options) -> str:
        python_version = await GenDockerImage.execute_and_get_line(
            command=f'{await cls.get_python_executable(options)} --version',
            options=options,
        )
        
        log.debug(f'{python_version = }')
        
        return python_version

    @classmethod
    @memoize
    async def get_remote_paths(cls, options: Options) -> FrozenSet[Path]:
        remote_paths = (
            await GenDockerContainer.get_environment(options)
        )['PYTHONPATH']
        
        remote_paths = remote_paths.split(':')
        remote_paths = [Path(remote_path) for remote_path in remote_paths]
        remote_paths = [
            remote_path for remote_path in remote_paths
            if options.workspace_path in remote_path.parents
        ]
        remote_paths = frozenset(remote_paths)

        log.debug(f'{remote_paths = }')
        
        return remote_paths

    @classmethod
    async def get_remote_address(cls, options) -> str:
        return f'{getpass.getuser()}@localhost:{await GenDockerContainer.get_ssh_port(options)}'

    @classmethod
    async def get_uri(cls, options: Options) -> str:
        return f'{await cls.get_remote_address(options)}{await cls.get_python_executable(options)}'

    @classmethod
    async def get_sftp_uri(cls, options: Options) -> str:
        return f'sftp://{await cls.get_uri(options)}'

    @classmethod
    async def get_ssh_uri(cls, options: Options) -> str:
        return f'ssh://{await cls.get_uri(options)}'

    @classmethod
    async def get_python_name(cls, options: Options) -> str:
        return options.idea_pycharm_webservers_name
        #return (
        #    f'{options.idea_pycharm_webservers_name} {await cls.get_python_version(options)} '
        #    f'({await cls.get_sftp_uri(options)})'
        #)

    @classmethod
    async def get_element(cls, options: Options) -> _Element:
        return etree.fromstring(
            parser=etree.XMLParser(remove_blank_text=True),
            text=dedent(f'''
                <application>
                  <component name="ProjectJdkTable">
                    <jdk version="2">
                      <name value="{await cls.get_python_name(options)}" />
                      <type value="Python SDK" />
                      <version value="{await cls.get_python_version(options)}" />
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
                          WEB_SERVER_CONFIG_ID="{options.idea_uuid}" 
                          WEB_SERVER_CONFIG_NAME="{await cls.get_remote_address(options)}" 
                          WEB_SERVER_CREDENTIALS_ID="{await cls.get_sftp_uri(options)}" 
                          INTERPRETER_PATH="{await cls.get_python_executable(options)}"
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

    @classmethod
    async def main(cls, options: Options) -> None:
        root_element = merge_elements(
            from_element=await cls.get_element(options),
            into_element=get_root_element_from_path(
                options.idea_pycharm_jdk_table_xml_path
            )
        )

        GenHost.write_text(
            data=etree.tostring(root_element, pretty_print=True, encoding=str),
            options=options,
            path=options.idea_pycharm_jdk_table_xml_path,
        )
