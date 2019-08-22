from dataclasses import dataclass, field
from logging import getLogger
from lxml import etree
# noinspection PyProtectedMember
from lxml.etree import _Element
from textwrap import dedent
from typing import Tuple, Type

from rosdev.gen.docker.ssh.port import GenDockerSshPort
from rosdev.gen.idea.ide.name import GenIdeaIdeName
from rosdev.gen.idea.universal import GenIdeaUniversal
from rosdev.gen.idea.uuid import GenIdeaUuid
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.xml import get_root_element_from_path, merge_elements


log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaClionWebserversXml(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenDockerSshPort,
        GenIdeaIdeName,
        GenIdeaUniversal,
        GenIdeaUuid,
    ))

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        # TODO py38 debug print
        log.debug(
            f'idea_webservers_xml_universal_path: {options.idea_clion_webservers_xml_universal_path}'
        )
        log.debug(
            f'idea_webservers_xml_workspace_path: {options.idea_webservers_xml_workspace_path}'
        )

    @classmethod
    def get_element(cls, options: Options) -> _Element:
        return etree.fromstring(
            parser=etree.XMLParser(remove_blank_text=True),
            text=dedent(f'''
                <application>
                  <component name="WebServers">
                    <option name="servers">
                      <webServer
                          id="{options.idea_uuid}"
                          name="{options.idea_clion_webservers_name}" 
                          url="http:///">
                        <fileTransfer
                            host="localhost"
                            port="{options.docker_ssh_port}"
                            privateKey="$USER_HOME$/.ssh/id_rsa" 
                            accessType="SFTP"
                            keyPair="true">
                          <option name="port" value="{options.docker_ssh_port}" />
                        </fileTransfer>
                      </webServer>
                    </option>
                  </component>
                </application>
            ''').strip()
        )

    @classmethod
    async def main(cls, options: Options) -> None:
        root_element = merge_elements(
            from_elements=cls.get_element(options),
            into_elements=get_root_element_from_path(
                options.idea_clion_webservers_xml_universal_path
            )
        )

        options.write_text(
            path=options.idea_clion_webservers_xml_universal_path,
            text=etree.tostring(root_element, pretty_print=True, encoding=str)
        )
