from dataclasses import dataclass, field
from logging import getLogger
from lxml import etree
# noinspection PyProtectedMember
from lxml.etree import _Element
from textwrap import dedent
from typing import Tuple, Type

from rosdev.gen.idea.base import GenIdeaBase
from rosdev.gen.idea.uuid import GenIdeaUuid
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.xml import get_root_element_from_path, merge_elements


log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaPycharmWebserversXml(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenIdeaBase,
        GenIdeaUuid,
    ))

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        # FIXME py38 debug print
        log.debug(
            f'idea_pycharm_webservers_xml_workspace_path: '
            f'{options.idea_pycharm_webservers_xml_workspace_path}'
        )

    @classmethod
    def get_element(cls, options: Options) -> _Element:
        return etree.fromstring(
            parser=etree.XMLParser(remove_blank_text=True),
            text=dedent(f'''
                <project version="4">
                  <component name="WebServers">
                    <option name="servers">
                      <webServer 
                          id="{options.idea_uuid}"
                          name="{options.idea_pycharm_webservers_name}" 
                          url="http://localhost">
                        <fileTransfer
                            host="localhost"
                            port="{options.docker_ssh_port}"
                            privateKey="$USER_HOME$/.ssh/id_rsa" 
                            accessType="SFTP"
                            keyPair="true">
                          <advancedOptions>
                            <advancedOptions
                                dataProtectionLevel="Private"
                                passiveMode="true" 
                                shareSSLContext="true" />
                          </advancedOptions>
                          <option name="port" value="{options.docker_ssh_port}" />
                        </fileTransfer>
                      </webServer>
                    </option>
                  </component>
                </project>
            ''').lstrip()
        )

    @classmethod
    async def main(cls, options: Options) -> None:
        root_element = merge_elements(
            from_element=cls.get_element(options),
            into_element=get_root_element_from_path(
                options.idea_pycharm_webservers_xml_workspace_path
            )
        )

        options.write_bytes(
            path=options.idea_pycharm_webservers_xml_workspace_path,
            text=etree.tostring(
                root_element, pretty_print=True, xml_declaration=True, encoding='UTF-8')
        )