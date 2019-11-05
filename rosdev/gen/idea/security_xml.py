from dataclasses import dataclass, field
from logging import getLogger
from lxml import etree
# noinspection PyProtectedMember
from lxml.etree import _Element
from textwrap import dedent
from typing import Tuple, Type

from rosdev.gen.host import GenHost
from rosdev.gen.idea.ide.name import GenIdeaIdeName
from rosdev.gen.idea.universal import GenIdeaUniversal
from rosdev.gen.idea.uuid import GenIdeaUuid
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.xml import get_root_element_from_path, merge_elements


log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaSecurityXml(Handler):
    
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenHost,
        GenIdeaIdeName,
        GenIdeaUniversal,
        GenIdeaUuid,
    ))

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        # TODO py38 debug print
        log.debug(f'idea_security_xml_universal_path: {options.idea_security_xml_path}')

    # noinspection PyUnusedLocal
    @classmethod
    def get_element(cls, options: Options) -> _Element:
        return etree.fromstring(
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

    @classmethod
    async def main(cls, options: Options) -> None:
        root_element = merge_elements(
            from_element=cls.get_element(options),
            into_element=get_root_element_from_path(options.idea_security_xml_path)
        )

        GenHost.write_text(
            data=etree.tostring(root_element, pretty_print=True, encoding=str),
            options=options,
            path=options.idea_security_xml_path,
        )
