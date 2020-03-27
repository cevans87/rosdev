from dataclasses import dataclass
from logging import getLogger
from lxml import etree
from textwrap import dedent

from rosdev.gen.host import GenHost
from rosdev.gen.idea.pycharm.webservers_xml import GenIdeaPycharmWebserversXml
from rosdev.gen.idea.workspace import GenIdeaWorkspace
from rosdev.util.atools import memoize
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.path import Path


log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaPycharmMiscXml(Handler):

    @classmethod
    @memoize
    async def get_bytes(cls, options: Options) -> bytes:
        element = etree.fromstring(
            parser=etree.XMLParser(remove_blank_text=True),
            text=dedent(f'''
                <project version="4">
                  <component name="JavaScriptSettings">
                    <option name="languageLevel" value="ES6" />
                  </component>
                  <component
                      name="ProjectRootManager"
                      version="2"
                      project-jdk-name="{await GenIdeaPycharmWebserversXml.get_name(options)}"
                      project-jdk-type="Python SDK" />
                </project>
            ''').lstrip()
        )
        # noinspection PyShadowingBuiltins
        bytes = etree.tostring(element, pretty_print=True, xml_declaration=True, encoding='UTF-8')

        log.debug(f'{cls.__name__} {bytes = }')

        return bytes

    @classmethod
    @memoize
    async def get_path(cls, options: Options) -> Path:
        path = await GenIdeaWorkspace.get_path(options) / 'misc.xml'

        log.debug(f'{cls.__name__} {path}')

        return path

    @classmethod
    async def main(cls, options: Options) -> None:
        GenHost.write_bytes(
            data=await cls.get_bytes(options),
            options=options,
            path=await cls.get_path(options),
        )
