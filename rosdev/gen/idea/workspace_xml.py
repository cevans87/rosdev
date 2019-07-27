from dataclasses import dataclass, field, replace
from logging import getLogger
from lxml import etree
# noinspection PyProtectedMember
from lxml.etree import _Element
from textwrap import dedent
from typing import Tuple, Type

from rosdev.gen.idea.base import GenIdeaBase
from rosdev.gen.idea.uuid import GenIdeaUuid
from rosdev.gen.ros.environment import GenRosEnvironment
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.xml import get_root_element_from_path, merge_elements
from rosdev.util.subprocess import exec


log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaWorkspaceXml(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenIdeaBase,
        GenIdeaUuid,
        GenRosEnvironment,
    ))

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        idea_workspace_xml_workspace_path = options.resolve_path(
            options.idea_workspace_xml_workspace_path
        )
        
        return replace(
            options,
            idea_workspace_xml_workspace_path=idea_workspace_xml_workspace_path,
        )

    @classmethod
    def get_element(cls, options: Options) -> _Element:
        return etree.fromstring(
            parser=etree.XMLParser(remove_blank_text=True),
            text=dedent(f'''
                <project version="4">
                  <component name="CMakeSettings">
                    <configurations>
                      <configuration
                          PROFILE_NAME="{options.build_type}"
                          CONFIG_NAME="{options.build_type}">
                        <ADDITIONAL_GENERATION_ENVIRONMENT>
                          <envs>
                            {''.join([
                                f'<env name="{k}" value="{v}" />' 
                                for k, v in options.ros_container_environment.items()
                            ])}
                          </envs>
                        </ADDITIONAL_GENERATION_ENVIRONMENT>
                      </configuration>
                    </configurations>
                  </component>
                </project>
            ''').strip()
        )

    @classmethod
    async def main(cls, options: Options) -> None:
        root_element = merge_elements(
            from_element=cls.get_element(options),
            into_element=get_root_element_from_path(options.idea_workspace_xml_workspace_path)
        )

        await exec(f'mkdir -p {options.idea_workspace_xml_workspace_path.parent}')
        with open(str(options.idea_workspace_xml_workspace_path), 'wb') as f_out:
            f_out.write(
                etree.tostring(
                    root_element, pretty_print=True, xml_declaration=True, encoding='UTF-8'
                )
            )
