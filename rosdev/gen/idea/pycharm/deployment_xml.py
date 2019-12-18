from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from lxml import etree
from textwrap import dedent

from rosdev.gen.host import GenHost
from rosdev.gen.idea.pycharm.jdk_table_xml import GenIdeaPycharmJdkTableXml
from rosdev.gen.idea.workspace import GenIdeaWorkspace
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.path import Path

log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaPycharmDeploymentXml(Handler):

    @classmethod
    async def get_bytes(cls, options: Options) -> bytes:
        element = etree.fromstring(
            parser=etree.XMLParser(remove_blank_text=True),
            text=dedent(f'''
                <project version="4">
                  <component name="PublishConfigData">
                    <serverData>
                      <paths name="{await GenIdeaPycharmJdkTableXml.get_remote_address(options)}">
                        <serverdata>
                          <mappings>
                            <mapping
                                deploy="{Path.workspace()}" 
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
