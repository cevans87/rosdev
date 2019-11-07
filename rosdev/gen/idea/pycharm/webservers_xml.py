from dataclasses import dataclass, field
from logging import getLogger
from lxml import etree
# noinspection PyProtectedMember
from lxml.etree import _Element
from textwrap import dedent
from typing import Tuple, Type

from rosdev.gen.docker.container import GenDockerContainer
from rosdev.gen.host import GenHost
from rosdev.gen.idea.base import GenIdeaBase
from rosdev.gen.idea.pycharm.jdk_table_xml import GenIdeaPycharmJdkTableXml
from rosdev.gen.idea.uuid import GenIdeaUuid
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.xml import get_root_element_from_path, merge_elements


log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaPycharmWebserversXml(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenDockerContainer,
        GenHost,
        GenIdeaBase,
        GenIdeaPycharmJdkTableXml,
        GenIdeaUuid,
    ))

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        log.debug(f'{options.idea_pycharm_webservers_xml_path = }')

    @classmethod
    async def get_element(cls, options: Options) -> _Element:
        from_element = etree.fromstring(
            parser=etree.XMLParser(remove_blank_text=True),
            text=dedent(f'''
                <application>
                  <component name="WebServers">
                    <option name="servers">
                      <webServer
                          id="{options.idea_uuid}"
                          name="{options.idea_pycharm_webservers_name}" 
                          url="http:///">
                        <fileTransfer
                            host="localhost"
                            port="{await GenDockerContainer.get_ssh_port(options)}"
                            privateKey="$USER_HOME$/.ssh/id_rsa"
                            accessType="SFTP"
                            keyPair="true">
                          <option
                              name="port"
                              value="{await GenDockerContainer.get_ssh_port(options)}" />
                        </fileTransfer>
                      </webServer>
                    </option>
                  </component>
                </application>
            ''').lstrip()
        )
        into_element = get_root_element_from_path(
            options.idea_pycharm_webservers_xml_path
        )
        element = merge_elements(from_element=from_element, into_element=into_element)
        
        log.debug(f'{element = }')
        
        return element

    @classmethod
    async def main(cls, options: Options) -> None:
        GenHost.write_bytes(
            data=etree.tostring(
                await cls.get_element(options),
                pretty_print=True,
                xml_declaration=True,
                encoding='UTF-8',
            ),
            options=options,
            path=options.idea_pycharm_webservers_xml_path,
        )
