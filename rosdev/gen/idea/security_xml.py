from dataclasses import dataclass, field, replace
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
from rosdev.util.subprocess import exec


log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaSecurityXml(Handler):
    
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenIdeaBase,
        GenIdeaUuid,
    ))

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        idea_security_xml_universal_path = options.resolve_path(
            options.idea_security_xml_universal_path
        )

        return replace(
            options,
            idea_security_xml_universal_path=idea_security_xml_universal_path,
        )

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
            into_element=get_root_element_from_path(options.idea_security_xml_universal_path)
        )

        await exec(f'mkdir -p {options.idea_security_xml_universal_path.parent}')
        with open(str(options.idea_security_xml_universal_path), 'w') as f_out:
            f_out.write(etree.tostring(root_element, pretty_print=True, encoding=str))
