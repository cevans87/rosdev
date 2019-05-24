from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from lxml import etree
# noinspection PyProtectedMember
from lxml.etree import _Element
from textwrap import dedent

from rosdev.gen.clion.config import Config as ClionConfig
from rosdev.gen.clion.uuid import Uuid
from rosdev.util.handler import Handler
from rosdev.util.xml import get_root_element_from_path, merge_elements
from rosdev.util.subprocess import exec


log = getLogger(__name__)


@memoize
@dataclass(frozen=True)
class CppToolchainsXml(Handler):

    @property
    def global_path_base(self) -> str:
        return f'{ClionConfig(self.options).global_path}/options'

    @property
    def global_path(self) -> str:
        return f'{self.global_path_base}/cpp.toolchains.xml'

    @property
    def element(self) -> _Element:
        return etree.fromstring(
            parser=etree.XMLParser(remove_blank_text=True),
            text=dedent(f'''
                <application>
                  <component name="CPPToolchains" version="4">
                    <toolchains detectedVersion="5">
                      <toolchain
                          name="rosdev"
                          toolSetKind="REMOTE"
                          customCMakePath="/usr/bin/cmake"
                          hostId="{Uuid(self.options).uuid}"
                          debuggerKind="CUSTOM_GDB"
                          customGDBPath="/usr/bin/gdb"
                      />
                    </toolchains>
                  </component>
                </application>
            ''').lstrip()
        )

    @memoize
    async def _main(self) -> None:
        root_element = merge_elements(
            from_element=self.element,
            into_element=get_root_element_from_path(self.global_path)
        )

        await exec(f'mkdir -p {self.global_path_base}')
        with open(self.global_path, 'w') as cpp_toolchains_xml_f_out:
            cpp_toolchains_xml_f_out.write(
                etree.tostring(root_element, pretty_print=True, encoding=str)
            )
