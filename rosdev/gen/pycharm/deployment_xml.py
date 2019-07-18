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
class DeploymentXml(Handler):

    @property
    def local_path_base(self) -> str:
        return ClionConfig(self.options).local_path

    @property
    def local_path(self) -> str:
        return f'{self.local_path_base}/deployment.xml'

    @property
    def element(self) -> _Element:
        return etree.fromstring(
            parser=etree.XMLParser(remove_blank_text=True),
            text=dedent(f'''
                <project version="4">
                  <component
                      name="PublishConfigData"
                      serverName="rosdev ({Uuid(self.options).uuid})">
                    <serverData>
                      <paths name="rosdev ({Uuid(self.options).uuid})">
                        <serverdata>
                          <mappings>
                            <mapping deploy="/" local="/" web="/" />
                          </mappings>
                          <excludedPaths>
                            <excludedPath path="/" />
                            <excludedPath local="true" path="/" />
                          </excludedPaths>
                        </serverdata>
                      </paths>
                    </serverData>
                  </component>
                </project>
            ''').strip()
        )

    @memoize
    async def _main(self) -> None:
        root_element = merge_elements(
            from_element=self.element,
            into_element=get_root_element_from_path(self.local_path)
        )

        await exec(f'mkdir -p {self.local_path_base}')
        with open(self.local_path, 'wb') as deployment_xml_f_out:
            deployment_xml_f_out.write(
                etree.tostring(
                    root_element, pretty_print=True, xml_declaration=True, encoding='UTF-8'
                )
            )
