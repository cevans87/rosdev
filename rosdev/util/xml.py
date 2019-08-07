from __future__ import annotations
from dataclasses import dataclass
from frozendict import frozendict
from logging import getLogger
from lxml import etree
# noinspection PyProtectedMember
from lxml.etree import Element, _Element
from pathlib import Path
import re
from typing import Dict, Mapping, NewType, Optional


log = getLogger(__name__)


_FIND_UUID_REGEX = re.compile(
    r'[\da-f]{8}-'
    r'[\da-f]{4}-'
    r'[\da-f]{4}-'
    r'[\da-f]{4}-'
    r'[\da-f]{12}'
)


@dataclass(frozen=True)
class _ElementKey:
    tag: str
    attrib: Mapping[str, str] = frozendict()

    @staticmethod
    def new(element: _Element) -> _ElementKey:
        return _ElementKey(
            tag=element.tag,
            attrib=frozendict({
                k: _FIND_UUID_REGEX.sub('XXXX', v) for k, v in element.attrib.items()
                if k not in {'value'}
            })
        )


def get_root_element_from_path(path: Path) -> Optional[_Element]:
    parser = etree.XMLParser(remove_blank_text=True)
    try:
        root_element: Optional[_Element] = etree.parse(str(path), parser).getroot()
    except OSError:
        root_element: Optional[_Element] = None

    return root_element


def merge_elements(from_element: Optional[_Element], into_element: Optional[_Element]) -> _Element:
    assert from_element is not None or into_element is not None

    if from_element is None:
        from_element = Element(into_element.tag, into_element.attrib)
    elif into_element is None:
        into_element = Element(from_element.tag, from_element.attrib)

    assert _ElementKey.new(from_element) == _ElementKey.new(into_element)

    # If we're merging a from_element and a to_element that are both associated with a
    # rosdev-generated uuid, only keep the from_element.
    if any(_FIND_UUID_REGEX.match(k) is not None for k in into_element.attrib.keys()):
        into_element = Element(from_element.tag, from_element.attrib)

    element: _Element = Element(from_element.tag, from_element.attrib)

    from_child_element_by_key: Dict[_ElementKey, _Element] = {}
    for from_child_element in from_element:
        from_child_element_by_key[_ElementKey.new(from_child_element)] = (
            from_child_element
        )

    into_child_element_by_key: Dict[_ElementKey, _Element] = {}
    for into_child_element in into_element:
        into_child_element_by_key[_ElementKey.new(into_child_element)] = (
            into_child_element
        )

    for keys in (
            from_child_element_by_key.keys() - into_child_element_by_key.keys(),
            from_child_element_by_key.keys() & into_child_element_by_key.keys(),
            into_child_element_by_key.keys() - from_child_element_by_key.keys()
    ):
        for key in keys:
            element.append(
                merge_elements(
                    from_element=from_child_element_by_key.get(key),
                    into_element=into_child_element_by_key.get(key)
                )
            )

    return element
