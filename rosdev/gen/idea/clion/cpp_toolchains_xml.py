from dataclasses import dataclass, field
from logging import getLogger
from lxml import etree
# noinspection PyProtectedMember
from lxml.etree import _Element
from textwrap import dedent
from typing import Tuple, Type

from rosdev.gen.idea.base import GenIdeaBase
from rosdev.gen.idea.uuid import GenIdeaUuid
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.xml import get_root_element_from_path, merge_elements


log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaClionCppToolchainsXml(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenIdeaBase,
        GenIdeaUuid,
    ))

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        # FIXME py38 debug print
        log.debug(
            f'idea_clion_cpp_toolchains_xml_universal_path: '
            f'{options.idea_clion_cpp_toolchains_xml_universal_path}'
        )

    @classmethod
    def get_element(cls, options: Options) -> _Element:
        return etree.fromstring(
            parser=etree.XMLParser(remove_blank_text=True),
            text=dedent(f'''
                <application>
                  <component name="CPPToolchains" version="4">
                    <toolchains detectedVersion="5">
                      <toolchain
                          name="{options.idea_clion_webservers_name}"
                          toolSetKind="REMOTE"
                          customCMakePath="/usr/bin/cmake"
                          hostId="{options.idea_uuid}"
                          debuggerKind="CUSTOM_GDB"
                          customGDBPath="/usr/bin/gdb"
                      />
                    </toolchains>
                  </component>
                </application>
            ''').lstrip()
        )

    @classmethod
    async def main(cls, options: Options) -> None:
        root_element = merge_elements(
            from_element=cls.get_element(options),
            into_element=get_root_element_from_path(
                options.idea_clion_cpp_toolchains_xml_universal_path
            )
        )

        options.write_text(
            path=options.idea_clion_cpp_toolchains_xml_universal_path,
            text=etree.tostring(root_element, pretty_print=True, encoding=str)
        )
