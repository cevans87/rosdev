from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from lxml import etree
from pathlib import Path
from textwrap import dedent

from rosdev.gen.host import GenHost
from rosdev.gen.idea.workspace import GenIdeaWorkspace
from rosdev.gen.idea.pycharm.webservers_xml import GenIdeaPycharmWebserversXml
from rosdev.gen.workspace import GenWorkspace
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.xml import get_root_element_from_path, merge_elements


log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaPycharmIml(Handler):

    @classmethod
    @memoize
    async def get_bytes(cls, options: Options) -> bytes:
        from_element = etree.fromstring(
            parser=etree.XMLParser(remove_blank_text=True),
            text=dedent(f'''
                <module type="PYTHON_MODULE" version="4">
                  <component name="NewModuleRootManager">
                    <content url="file://$MODULE_DIR$" />
                    <orderEntry
                        type="jdk"
                        jdkName="{await GenIdeaPycharmWebserversXml.get_name(options)}" 
                        jdkType="Python SDK" />
                    <orderEntry type="sourceFolder" forTests="false" />
                  </component>
                  <component name="PyDocumentationSettings">
                    <option name="format" value="REST" />
                    <option name="myDocStringFormat" value="reStructuredText" />
                  </component>
                  <component name="TestRunnerService">
                    <option name="PROJECT_TEST_RUNNER" value="pytest" />
                  </component>
                </module>
            ''').lstrip()
        )
        into_element = get_root_element_from_path(await cls.get_path(options))
        element = merge_elements(from_element=from_element, into_element=into_element)
        # noinspection PyShadowingBuiltins
        bytes = etree.tostring(element, pretty_print=True, xml_declaration=True, encoding='UTF-8')

        log.debug(f'{cls.__name__} {bytes = }')

        return bytes

    @classmethod
    @memoize
    async def get_path(cls, options: Options) -> Path:
        path = (
                await GenIdeaWorkspace.get_path(options) /
                f'{(await GenWorkspace.get_path(options)).parts[-1]}.iml'
        )

        log.debug(f'{cls.__name__} {path}')

        return path

    @classmethod
    async def main(cls, options: Options) -> None:
        GenHost.write_bytes(
            data=await cls.get_bytes(options),
            options=options,
            path=await cls.get_path(options),
        )
