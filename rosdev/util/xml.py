from __future__ import annotations
from dataclasses import dataclass
from frozendict import frozendict
import getpass
from logging import getLogger
from lxml import etree
# noinspection PyProtectedMember
from lxml.etree import Element, _Element
from pathlib import Path
import re
from textwrap import dedent
from typing import Dict, FrozenSet, List, Mapping, Optional


log = getLogger(__name__)


_FIND_NUMBER_REGEX = re.compile(r'^\d+$')
_FIND_UUID_REGEX = re.compile(r'[\da-f]{8}-[\da-f]{4}-[\da-f]{4}-[\da-f]{4}-[\da-f]{12}')
_FIND_REMOTE_ADDRESS_REGEX = re.compile(fr'^{getpass.getuser()}@localhost:\d+$')
_FIND_ROSDEV_REGEX = re.compile(r'^rosdev.*$')
_FIND_DEDUP_REGEX = re.compile(
    f'(?:{_FIND_NUMBER_REGEX.pattern})|'
    f'(?:{_FIND_UUID_REGEX.pattern})|'
    f'(?:{_FIND_REMOTE_ADDRESS_REGEX.pattern})'
    f'(?:{_FIND_ROSDEV_REGEX.pattern})'
)


@dataclass(frozen=True)
class _ElementKey:
    tag: str
    attrib: Mapping[str, str] = frozendict()
    child_keys: FrozenSet[_ElementKey] = frozenset()

    @staticmethod
    def new(element: _Element) -> _ElementKey:
        if element.getparent() is None or element.getparent().tag != 'component':
            child_keys = frozenset()
        else:
            child_keys = frozenset({
                _ElementKey.new(child_element)
                for child_element in element
                if child_element.tag in {'name', 'webServer'}
            })

        return _ElementKey(
            tag=element.tag,
            attrib=frozendict({
                #k: _FIND_UUID_REGEX.sub('XXXX', v)
                k: _FIND_DEDUP_REGEX.sub('XXXX', v)
                for k, v in element.attrib.items()
                if (element.tag, k) not in {
                    ('option', 'value'),
                    ('env', 'value'),
                }
            }),
            child_keys=child_keys,
        )


def get_root_element_from_path(path: Path) -> Optional[_Element]:
    parser = etree.XMLParser(remove_blank_text=True)
    try:
        root_element: Optional[_Element] = etree.parse(str(path), parser).getroot()
    except OSError:
        root_element: Optional[_Element] = None

    return root_element


def get_element_from_text(text: str) -> Optional[_Element]:
    text = dedent(text).strip()
    parser = etree.XMLParser(remove_blank_text=True)
    try:
        root_element: Optional[_Element] = etree.fromstring(text, parser)
    except OSError:
        root_element: Optional[_Element] = None

    return root_element


def get_text_from_element(element: _Element) -> str:
    return etree.tostring(element, pretty_print=True, encoding=str)


def get_text_from_text(text: str) -> str:
    return get_text_from_element(get_element_from_text(text))


def merge_elements(from_element: Optional[_Element], into_element: Optional[_Element]) -> _Element:
    assert from_element is not None or into_element is not None

    if from_element is None:
        from_element = into_element
    elif (into_element is None) or (into_element.tag in {'jdk', 'webServer'}):
        into_element = from_element

    assert _ElementKey.new(from_element) == _ElementKey.new(into_element)

    # If we're merging a from_element and a to_element that are both associated with a
    # rosdev-generated uuid, only keep the from_element.
    if any(_FIND_UUID_REGEX.match(k) is not None for k in into_element.attrib.keys()):
        into_element = Element(from_element.tag, from_element.attrib)

    element: _Element = Element(from_element.tag, from_element.attrib)

    from_child_element_by_key: Dict[_ElementKey, _Element] = {}
    for from_child_element in from_element:
        from_child_element_by_key[_ElementKey.new(from_child_element)] = from_child_element

    into_child_element_by_key: Dict[_ElementKey, _Element] = {}
    for into_child_element in into_element:
        into_child_element_by_key[_ElementKey.new(into_child_element)] = into_child_element

    ordered_keys: List[_ElementKey] = []
    for key in from_child_element_by_key:
        if key not in into_child_element_by_key:
            ordered_keys.append(key)

    for key in from_child_element_by_key:
        if key in into_child_element_by_key:
            ordered_keys.append(key)

    for key in into_child_element_by_key:
        if key not in from_child_element_by_key:
            ordered_keys.append(key)

    for key in ordered_keys:
        element.append(
            merge_elements(
                from_element=from_child_element_by_key.get(key),
                into_element=into_child_element_by_key.get(key),
            )
        )

    return element
