from dataclasses import dataclass, field
from logging import getLogger
from lxml import etree
# noinspection PyProtectedMember
from lxml.etree import _Element
from textwrap import dedent
from typing import Tuple, Type

from rosdev.gen.idea.ide.name import GenIdeaIdeName
from rosdev.gen.idea.universal import GenIdeaUniversal
from rosdev.gen.idea.uuid import GenIdeaUuid
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.xml import get_root_element_from_path, merge_elements

log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaClionDeploymentXml(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenIdeaIdeName,
        GenIdeaUniversal,
        GenIdeaUuid,
    ))

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        # FIXME py38 debug print
        log.debug(
            f'pycharm_deployment_xml_workspace_path: '
            f'{options.idea_pycharm_deployment_xml_workspace_path}'
        )

    @classmethod
    def get_element(cls, options: Options) -> _Element:
        return etree.fromstring(
            parser=etree.XMLParser(remove_blank_text=True),
            text=dedent(f'''
                <project version="4">
                  <component name="PublishConfigData" serverName="rosdev_{options.idea_ide_name}">
                    <serverData>
                      <paths name="rosdev_{options.idea_ide_name}">
                        <serverdata>
                          <mappings>
                            <mapping deploy="/" local="/" web="/" />
                          </mappings>
                          <excludedPaths>
                            <excludedPath local="true" path="/" />
                            <excludedPath path="/" />
                          </excludedPaths>
                        </serverdata>
                      </paths>
                    </serverData>
                  </component>
                </project>
            ''').strip()
        )

    @classmethod
    async def main(cls, options: Options) -> None:
        root_element = merge_elements(
            from_element=cls.get_element(options),
            into_element=get_root_element_from_path(
                options.idea_pycharm_deployment_xml_workspace_path
            )
        )

        options.write_bytes(
            path=options.idea_pycharm_deployment_xml_workspace_path,
            text=etree.tostring(
                root_element, pretty_print=True, xml_declaration=True, encoding='UTF-8'
            )
        )
