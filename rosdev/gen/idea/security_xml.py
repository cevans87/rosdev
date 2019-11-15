from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from lxml import etree
from pathlib import Path
from textwrap import dedent

from rosdev.gen.idea.home import GenIdeaHome
from rosdev.gen.host import GenHost
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.xml import get_root_element_from_path, merge_elements


log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaSecurityXml(Handler):

    @classmethod
    @memoize
    async def get_text(cls, options: Options) -> str:
        from_element = etree.fromstring(
            parser=etree.XMLParser(remove_blank_text=True),
            text=dedent(f'''
                <application>
                  <component name="PasswordSafe">
                    <option name="PROVIDER" value="KEEPASS" />
                    <option name="rememberPasswordByDefault" value="false" />
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
    @memoize
    async def get_path(cls, options: Options) -> Path:
        return await GenIdeaHome.get_path(options) / 'options' / 'security.xml'

    @classmethod
    async def main(cls, options: Options) -> None:
        GenHost.write_text(
            data=await cls.get_text(options),
            options=options,
            path=await cls.get_path(options),
        )
