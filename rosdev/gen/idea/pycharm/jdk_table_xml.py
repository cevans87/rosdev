from dataclasses import dataclass, field
from logging import getLogger
from lxml import etree
# noinspection PyProtectedMember
from lxml.etree import _Element
import os
from textwrap import dedent
from typing import Tuple, Type

from rosdev.gen.docker.core import GenDockerCore
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
        GenDockerCore,
        GenIdeaIdeName,
        GenIdeaUniversal,
        GenIdeaUuid,
    ))

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        log.debug(f'{options.idea_pycharm_jdk_table_xml_path = }')

    @classmethod
    async def get_python_version(cls, options: Options) -> str:
        return await cls.execute_host_and_get_line(
            command=f'docker exec {options.docker_container_name} python --version'
        )

    @classmethod
    async def get_python_name(cls, options: Options) -> str:
        return (
            f'Remote Python {await cls.get_python_version(options)} '
            f'({options.idea_pycharm_jdk_table_xml_sftp_uri})'
        )

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
                      <homePath value="{options.idea_pycharm_jdk_table_xml_ssh_uri}" />
                      <additional
                          HOST="localhost"
                          PORT="{options.docker_ssh_port}"
                          USERNAME="{os.getlogin()}" 
                          PRIVATE_KEY_FILE="$USER_HOME$/.ssh/id_rsa"
                          USE_KEY_PAIR="true"
                          USE_AUTH_AGENT="false"
                          INTERPRETER_PATH="/usr/bin/python"
                          HELPERS_PATH="$USER_HOME$/.pycharm_helpers"
                          INITIALIZED="false"
                          VALID="true"
                          RUN_AS_ROOT_VIA_SUDO="false"
                          SKELETONS_PATH=""
                          VERSION="{await cls.get_python_version(options)}">
                        <PathMappingSettings>
                          <option name="pathMappings">
                            <list>
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

        options.write_text(
            path=options.idea_pycharm_jdk_table_xml_path,
            text=etree.tostring(root_element, pretty_print=True, encoding=str)
        )
