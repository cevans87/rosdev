from dataclasses import dataclass, field
from logging import getLogger
from lxml import etree
# noinspection PyProtectedMember
from lxml.etree import _Element
from textwrap import dedent
from typing import Tuple, Type

from rosdev.gen.environment import GenEnvironment
from rosdev.gen.idea.ide.name import GenIdeaIdeName
from rosdev.gen.idea.universal import GenIdeaUniversal
from rosdev.gen.idea.uuid import GenIdeaUuid
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.xml import get_root_element_from_path, merge_elements


log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaWorkspaceXml(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenEnvironment,
        GenIdeaIdeName,
        GenIdeaUniversal,
        GenIdeaUuid,
    ))

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        log.debug(f'{options.idea_workspace_xml_path = }')

    @classmethod
    async def get_element(cls, options: Options) -> _Element:
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
                                for k, v
                                in (await GenEnvironment.get_environment_container(options)).items()
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
            from_element=await cls.get_element(options),
            into_element=get_root_element_from_path(options.idea_workspace_xml_path)
        )

        options.write_bytes(
            path=options.idea_workspace_xml_path,
            text=etree.tostring(
                root_element, pretty_print=True, xml_declaration=True, encoding='UTF-8'
            )
        )
