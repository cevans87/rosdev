from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from lxml import etree
# noinspection PyProtectedMember
from lxml.etree import _Element
from textwrap import dedent

from rosdev.gen.clion.config import Config as ClionConfig
from rosdev.gen.clion.uuid import Uuid
from rosdev.util.handler import Handler
from rosdev.util.xml import get_root_element_from_path, merge_elements
from rosdev.util.subprocess import exec


log = getLogger(__name__)


@memoize
@dataclass(frozen=True)
class WebserversXml(Handler):

    @property
    def global_path_base(self) -> str:
        return f'{ClionConfig(self.options).global_path}/options'

    @property
    def global_path(self) -> str:
        return f'{self.global_path_base}/webServers.xml'

    @property
    def element(self) -> _Element:
        return etree.fromstring(
            parser=etree.XMLParser(remove_blank_text=True),
            text=dedent(f'''
                <application>
                  <component name="WebServers">
                    <option name="servers">
                      <webServer
                          id="{Uuid(self.options).uuid}"
                          name="rosdev ({Uuid(self.options).uuid})" 
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

    @memoize
    async def _main(self) -> None:
        root_element = merge_elements(
            from_element=self.element,
            into_element=get_root_element_from_path(self.global_path)
        )

        await exec(f'mkdir -p {self.global_path_base}')
        with open(self.global_path, 'w') as webservers_xml_f_out:
            webservers_xml_f_out.write(
                etree.tostring(root_element, pretty_print=True, encoding=str)
            )
