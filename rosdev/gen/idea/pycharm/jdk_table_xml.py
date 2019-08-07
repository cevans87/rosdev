from dataclasses import dataclass, field
from logging import getLogger
from lxml import etree
# noinspection PyProtectedMember
from lxml.etree import _Element
import os
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
class GenIdeaPycharmJdkTableXml(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenIdeaIdeName,
        GenIdeaUniversal,
        GenIdeaUuid,
    ))

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        # FIXME py38 debug print
        log.debug(
            f'pycharm_jdk_table_xml_universal_path: '
            f'{options.idea_pycharm_jdk_table_xml_universal_path}'
        )

    @classmethod
    async def get_element(cls, options: Options) -> _Element:
        return etree.fromstring(
            parser=etree.XMLParser(remove_blank_text=True),
            # FIXME
            text=dedent(f'''
                <application>
                  <component name="ProjectJdkTable">
                    <jdk version="2">
                      <name value="{options.idea_pycharm_project_jdk_name}" />
                      <type value="Python SDK" />
                      <version value="Python 2.7.15+" />
                      <homePath
                          value="ssh://{os.getlogin()}@localhost:{options.docker_ssh_port}/usr/bin/python" />
                      <additional HOST="localhost" PORT="34663" USERNAME="cevans" PRIVATE_KEY_FILE="$USER_HOME$/.ssh/id_rsa" USE_KEY_PAIR="true" USE_AUTH_AGENT="false" INTERPRETER_PATH="/usr/bin/python" HELPERS_PATH="$USER_HOME$/.pycharm_helpers" INITIALIZED="false" VALID="true" RUN_AS_ROOT_VIA_SUDO="false" SKELETONS_PATH="" VERSION="Python 2.7.15+">
                        <PathMappingSettings>
                          <option name="pathMappings">
                            <list>
                            </list>
                          </option>
                        </PathMappingSettings>
                      </additional>
                    </jdk>
                  </component>
                </application>
            ''').strip()
        )

    @classmethod
    async def main(cls, options: Options) -> None:
        root_element = merge_elements(
            from_element=cls.get_element(options),
            into_element=get_root_element_from_path(
                options.idea_pycharm_jdk_table_xml_universal_path
            )
        )

        options.write_text(
            path=options.idea_pycharm_jdk_table_xml_universal_path,
            text=etree.tostring(root_element, pretty_print=True, encoding=str)
        )
