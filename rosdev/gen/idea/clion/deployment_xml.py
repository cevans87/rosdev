from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from lxml import etree
from pathlib import Path
from textwrap import dedent

from rosdev.gen.host import GenHost
from rosdev.gen.idea.clion.webservers_xml import GenIdeaClionWebserversXml
from rosdev.gen.idea.workspace import GenIdeaWorkspace
from rosdev.gen.workspace import GenWorkspace
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.xml import get_root_element_from_path, merge_elements


log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaClionDeploymentXml(Handler):
    
    @classmethod
    @memoize
    async def get_bytes(cls, options: Options) -> bytes:
        element = etree.fromstring(
            parser=etree.XMLParser(remove_blank_text=True),
            text=dedent(f'''
                <project version="4">
                  <component name="PublishConfigData">
                    <serverData>
                      <paths name="{await GenIdeaClionWebserversXml.get_name(options)}">
                        <serverdata>
                          <mappings>
                            <mapping
                                deploy="{await GenWorkspace.get_path(options)}" 
                                local="$PROJECT_DIR$" />
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

        log.debug(f'{cls.__name__} {bytes = }')

        return bytes

    @classmethod
    @memoize
    async def get_path(cls, options: Options) -> Path:
        path = await GenIdeaWorkspace.get_path(options) / 'deployment.xml'

        log.debug(f'{cls.__name__} {path = }')

        return path

    @classmethod
    async def main(cls, options: Options) -> None:
        GenHost.write_bytes(
            data=await cls.get_bytes(options),
            options=options,
            path=await cls.get_path(options),
        )
