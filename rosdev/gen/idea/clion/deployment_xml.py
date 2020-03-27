from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from lxml import etree
from textwrap import dedent

from rosdev.gen.idea.clion.webservers_xml import GenIdeaClionWebserversXml
from rosdev.gen.idea.workspace import GenIdeaWorkspace
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.path import Path


log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaClionDeploymentXml(Handler):
    
    @staticmethod
    @memoize
    async def get_bytes(options: Options) -> bytes:
        element = etree.fromstring(
            parser=etree.XMLParser(remove_blank_text=True),
            text=dedent(f'''
                <project version="4">
                  <component name="PublishConfigData">
                    <serverData>
                      <paths name="{await GenIdeaClionWebserversXml.get_name(options)}">
                        <serverdata>
                          <mappings>
                            <mapping deploy="{Path.workspace()}" local="$PROJECT_DIR$" />
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
        # noinspection PyShadowingBuiltins
        bytes = etree.tostring(element, pretty_print=True, xml_declaration=True, encoding='UTF-8')

        log.debug(f'{__class__.__name__} {bytes = }')

        return bytes

    @staticmethod
    @memoize
    async def get_path(options: Options) -> Path:
        path = await GenIdeaWorkspace.get_path(options) / 'deployment.xml'

        log.debug(f'{__class__.__name__} {path = }')

        return path

    @staticmethod
    @memoize
    async def main(options: Options) -> None:
        (await GenIdeaClionDeploymentXml.get_path(options)).write_bytes(
            data=await GenIdeaClionDeploymentXml.get_bytes(options)
        )
