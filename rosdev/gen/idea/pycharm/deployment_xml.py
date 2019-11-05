from dataclasses import dataclass, field
from logging import getLogger
from lxml import etree
# noinspection PyProtectedMember
from lxml.etree import _Element
from textwrap import dedent
from typing import Tuple, Type

from rosdev.gen.docker.container import GenDockerContainer
from rosdev.gen.host import GenHost
from rosdev.gen.idea.base import GenIdeaBase
from rosdev.gen.idea.pycharm.jdk_table_xml import GenIdeaPycharmJdkTableXml
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.xml import get_root_element_from_path, merge_elements

log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaPycharmDeploymentXml(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenDockerContainer,
        GenHost,
        GenIdeaBase,
        GenIdeaPycharmJdkTableXml,
    ))

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        log.debug(f'{options.idea_pycharm_deployment_xml_path = }')

    @classmethod
    async def get_element(cls, options: Options) -> _Element:
        return etree.fromstring(
            parser=etree.XMLParser(remove_blank_text=True),
            text=dedent(f'''
                <project version="4">
                  <component name="PublishConfigData">
                    <serverData>
                      <paths name="{await GenIdeaPycharmJdkTableXml.get_remote_address(options)}">
                        <serverdata>
                          <mappings>
                            <mapping deploy="{options.workspace_path}" local="$PROJECT_DIR$" />
                          </mappings>
                          <excludedPaths>
                            <excludedPath path="{options.workspace_path}" />
                            <excludedPath local="true" path="$PROJECT_DIR$" />
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
            from_element=await cls.get_element(options),
            into_element=get_root_element_from_path(
                options.idea_pycharm_deployment_xml_path
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
            path=options.idea_pycharm_deployment_xml_path,
        )
