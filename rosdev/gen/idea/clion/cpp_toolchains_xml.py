from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from lxml import etree
from pathlib import Path
from textwrap import dedent

from rosdev.gen.idea.clion.webservers_xml import GenIdeaClionWebserversXml
from rosdev.gen.idea.home import GenIdeaHome
from rosdev.gen.idea.workspace import GenIdeaWorkspace
from rosdev.gen.host import GenHost
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.xml import get_root_element_from_path, merge_elements


log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaClionCppToolchainsXml(Handler):

    @classmethod
    @memoize
    async def get_path(cls, options: Options) -> Path:
        path = await GenIdeaHome.get_path(options) / 'options' / 'cpp.toolchains.xml'
        
        log.debug(f'{cls.__name__} {path = }')
        
        return path

    @classmethod
    @memoize
    async def get_text(cls, options: Options) -> str:
        from_element = etree.fromstring(
            parser=etree.XMLParser(remove_blank_text=True),
            text=dedent(f'''
                <application>
                  <component name="CPPToolchains" version="4">
                    <toolchains detectedVersion="5">
                      <toolchain
                          name="{await GenIdeaClionWebserversXml.get_name(options)}"
                          toolSetKind="REMOTE"
                          customCMakePath="/usr/bin/cmake"
                          hostId="{await GenIdeaWorkspace.get_uuid(options)}"
                          debuggerKind="CUSTOM_GDB"
                          customGDBPath="/usr/bin/gdb"
                      />
                    </toolchains>
                  </component>
                </application>
            ''').lstrip()
        )
        into_element = get_root_element_from_path(await cls.get_path(options))
        element = merge_elements(from_element=from_element, into_element=into_element)
        text = etree.tostring(element, pretty_print=True, encoding=str)

        log.debug(f'{cls.__name__} {text = }')

        return text

    @classmethod
    async def main(cls, options: Options) -> None:
        GenHost.write_text(
            data=await cls.get_text(options),
            options=options,
            path=await cls.get_path(options),
        )
