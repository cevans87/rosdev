from dataclasses import dataclass
# noinspection PyProtectedMember
from lxml.etree import _Element
import pytest

from rosdev.util.xml import (
    get_element_from_text, get_text_from_element, get_text_from_text, merge_elements
)


@dataclass(frozen=True)
class Fixture:
    element: _Element
    text: str


@pytest.fixture()
def single_item_from_fixture() -> Fixture:
    element = get_element_from_text('''
        <application>
          <component name="ProjectJdkTable">
            <jdk version="2">
              <name value="Remote Python 2.7.15+ (sftp://cevans@localhost:34663/usr/bin/python)" />
            </jdk>
          </component>
        </application>
    ''')
    text = get_text_from_element(element)
    
    return Fixture(element=element, text=text)


@pytest.fixture()
def empty_list_into_fixture() -> Fixture:
    element = get_element_from_text('''
        <application>
          <component name="ProjectJdkTable">
          </component>
        </application>
    ''')
    text = get_text_from_element(element)

    return Fixture(element=element, text=text)


def test_merge_new_component_creates_list(
        single_item_from_fixture: Fixture,
        empty_list_into_fixture: Fixture,
) -> None:
    merged_text = get_text_from_element(
        merge_elements(
            from_element=single_item_from_fixture.element,
            into_element=empty_list_into_fixture.element
        )
    )
    assert merged_text == get_text_from_text('''
        <application>
          <component name="ProjectJdkTable">
            <jdk version="2">
              <name value="Remote Python 2.7.15+ (sftp://cevans@localhost:34663/usr/bin/python)" />
            </jdk>
          </component>
        </application>
    ''')


@pytest.fixture()
def many_items_into_fixture() -> Fixture:
    element = get_element_from_text('''
        <application>
          <component name="ProjectJdkTable">
            <jdk version="2">
              <name value="Python 3.5 (untitled)" />
            </jdk>
            <jdk version="2">
              <name value="Python 3.7" />
            </jdk>
            <jdk version="2">
              <name value="Python 3.7 (2)" />
            </jdk>
            <jdk version="2">
              <name value="Python 3.6 (atools)" />
            </jdk>
            <jdk version="2">
              <name value="Python 3.8 (rosdev)" />
            </jdk>
            <jdk version="2">
              <name value="Python 3.7 (rosdev)" />
            </jdk>
          </component>
        </application>
    ''')
    text = get_text_from_element(element)
    
    return Fixture(element=element, text=text)


def test_merge_none_into_existing_component_list_preserves_list(
        many_items_into_fixture: Fixture
) -> None:
    merged_text = get_text_from_element(
        merge_elements(
            from_element=None,
            into_element=many_items_into_fixture.element
        )
    )
    assert merged_text == many_items_into_fixture.text


def test_merge_new_component_item_inserts_into_existing_component_list(
        single_item_from_fixture: Fixture,
        many_items_into_fixture: Fixture,
) -> None:
    merged_text = get_text_from_element(
        merge_elements(
            from_element=single_item_from_fixture.element,
            into_element=many_items_into_fixture.element
        )
    )
    assert merged_text == get_text_from_text('''
        <application>
          <component name="ProjectJdkTable">
            <jdk version="2">
              <name value="Remote Python 2.7.15+ (sftp://cevans@localhost:34663/usr/bin/python)" />
            </jdk>
            <jdk version="2">
              <name value="Python 3.5 (untitled)" />
            </jdk>
            <jdk version="2">
              <name value="Python 3.7" />
            </jdk>
            <jdk version="2">
              <name value="Python 3.7 (2)" />
            </jdk>
            <jdk version="2">
              <name value="Python 3.6 (atools)" />
            </jdk>
            <jdk version="2">
              <name value="Python 3.8 (rosdev)" />
            </jdk>
            <jdk version="2">
              <name value="Python 3.7 (rosdev)" />
            </jdk>
          </component>
        </application>
    ''')
