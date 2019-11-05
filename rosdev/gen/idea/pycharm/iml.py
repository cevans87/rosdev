from dataclasses import dataclass, field
from logging import getLogger
from lxml import etree
# noinspection PyProtectedMember
from lxml.etree import _Element
from textwrap import dedent
from typing import Tuple, Type

from rosdev.gen.host import GenHost
from rosdev.gen.idea.base import GenIdeaBase
from rosdev.gen.idea.pycharm.jdk_table_xml import GenIdeaPycharmJdkTableXml
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.xml import get_root_element_from_path, merge_elements


log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaPycharmIml(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenHost,
        GenIdeaBase,
        GenIdeaPycharmJdkTableXml,
    ))

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        log.debug(f'{options.idea_pycharm_misc_xml_path = }')

    @classmethod
    async def get_element(cls, options: Options) -> _Element:
        return etree.fromstring(
            parser=etree.XMLParser(remove_blank_text=True),
            text=dedent(f'''
                <module type="PYTHON_MODULE" version="4">
                  <component name="NewModuleRootManager">
                    <content url="file://$MODULE_DIR$" />
                    <orderEntry
                        type="jdk"
                        jdkName="{await GenIdeaPycharmJdkTableXml.get_python_name(options)}" 
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

    @classmethod
    async def main(cls, options: Options) -> None:
        root_element = merge_elements(
            from_element=await cls.get_element(options),
            into_element=get_root_element_from_path(
                options.idea_pycharm_iml_path
            )
        )

        GenHost.write_bytes(
            data=etree.tostring(
                root_element,
                pretty_print=True,
                xml_declaration=True,
                encoding='UTF-8',
            ),
            options=options,
            path=options.idea_pycharm_iml_path,
        )
