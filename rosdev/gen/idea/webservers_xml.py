from dataclasses import dataclass, field, replace
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
from rosdev.util.subprocess import exec


log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaWebserversXml(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenIdeaBase,
        GenIdeaUuid,
    ))

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        idea_webservers_xml_universal_path = options.resolve_path(
            options.idea_webservers_xml_universal_path
        )

        return replace(
            options,
            idea_webservers_xml_universal_path=idea_webservers_xml_universal_path,
        )

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        # TODO py38 debug string
        log.debug(
            f'idea_wbservers_xml_universal_path: {options.idea_webservers_xml_universal_path}'
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
                          name="rosdev_{options.idea_base_ide_name} ({options.idea_uuid})" 
                          url="http:///">
                        <fileTransfer
                            host="localhost"
                            port="22"
                            privateKey="$USER_HOME$/.ssh/id_rsa" 
                            accessType="SFTP"
                            keyPair="true">
                          <option name="port" value="22" />
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
            from_element=cls.get_element(options),
            into_element=get_root_element_from_path(options.idea_webservers_xml_universal_path)
        )

        await exec(f'mkdir -p {options.idea_webservers_xml_universal_path.parent}')
        with open(str(options.idea_webservers_xml_universal_path), 'w') as f_out:
            f_out.write(
                etree.tostring(root_element, pretty_print=True, encoding=str)
            )
