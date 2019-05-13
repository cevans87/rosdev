from __future__ import annotations
from asyncio import gather
from atools import memoize
from dataclasses import dataclass
from frozendict import frozendict
from logging import getLogger
from lxml import etree
# noinspection PyProtectedMember
from lxml.etree import Element, _Element
import os
from pathlib import Path
import re
import sys
from textwrap import dedent
from typing import Dict, Mapping, Optional

from rosdev.gen.clion.underlay import Underlay
from rosdev.util.handler import Handler
from rosdev.util.subprocess import exec


log = getLogger(__name__)

_FIND_UUID_REGEX = re.compile(
    r'[\da-f]{8}-'
    r'[\da-f]{4}-'
    r'[\da-f]{4}-'
    r'[\da-f]{4}-'
    r'[\da-f]{12}'
)


@dataclass(frozen=True)
class _Key:
    tag: str
    attrib: Mapping[str, str] = frozendict()

    @staticmethod
    def new(tag: str, attrib: Mapping[str, str] = frozendict()) -> _Key:
        return _Key(
            tag=tag,
            attrib=frozendict({
                k: _FIND_UUID_REGEX.sub('XXXX', v) for k, v in attrib.items()
            })
        )


@memoize
@dataclass(frozen=True)
class Settings(Handler):

    @property
    def global_path(self) -> str:
        # FIXME find a way to determine installed clion version.
        if sys.platform == 'darwin':
            return f'{Path.home()}/Library/Preferences/.CLion2019.1/config/options'
        return f'{Path.home()}/.CLion2019.1/config/options'

    @property
    def deployment_xml_path_base(self) -> str:
        return f'{os.getcwd()}/.idea'

    @property
    def deployment_xml_path(self) -> str:
        return f'{self.deployment_xml_path_base}/deployment.xml'

    @property
    def deployment_xml_element(self) -> _Element:
        return etree.fromstring(
            parser=self._xml_parser,
            text=dedent(f'''
                <project version="4">
                  <component name="PublishConfigData" serverName="rosdev ({self.uuid})">
                    <serverData>
                      <paths name="rosdev ({self.uuid})">
                        <serverdata>
                          <mappings>
                            <mapping deploy="/" local="/" web="/" />
                          </mappings>
                          <excludedPaths>
                            <excludedPath path="/" />
                            <excludedPath local="true" path="/" />
                          </excludedPaths>
                        </serverdata>
                      </paths>
                    </serverData>
                  </component>
                </project>
            ''').strip()
        )

    @property
    def cpp_toolchains_xml_path_base(self) -> str:
        return Underlay(self.options).options_path

    @property
    def cpp_toolchains_xml_path(self) -> str:
        return f'{self.cpp_toolchains_xml_path_base}/cpp.toolchains.xml'

    @property
    def cpp_toolchains_xml_element(self) -> _Element:
        return etree.fromstring(
            parser=self._xml_parser,
            text=dedent(f'''
                <application>
                  <component name="CPPToolchains" version="4">
                    <toolchains detectedVersion="5">
                      <toolchain
                          name="rosdev"
                          toolSetKind="REMOTE"
                          customCMakePath="/usr/bin/cmake"
                          hostId="{self.uuid}"
                          debuggerKind="CUSTOM_GDB"
                          customGDBPath="/usr/bin/gdb"
                      />
                    </toolchains>
                  </component>
                </application>
            ''').lstrip()
        )

    @property
    def webservers_xml_path_base(self) -> str:
        return self.global_path

    @property
    def webservers_xml_path(self) -> str:
        return f'{self.webservers_xml_path_base}/webServers.xml'

    @property
    def webservers_xml_element(self) -> _Element:
        return etree.fromstring(
            parser=self._xml_parser,
            text=dedent(f'''
                <application>
                  <component name="WebServers">
                    <option name="servers">
                      <webServer
                          id="{self.uuid}"
                          name="rosdev ({self.uuid})" 
                          url="http:///">
                        <fileTransfer
                            host="localhost"
                            port="22"
                            privateKey="$USER_HOME$/.ssh/id_rsa" 
                            accessType="SFTP"
                            keyPair="true">
                          <option name="port" value="22" />
                        </fileTransfer>
                      </webServer>
                    </option>
                  </component>
                </application>
            ''').strip()
        )

    @property
    def _xml_parser(self) -> etree.XMLParser:
        return etree.XMLParser(remove_blank_text=True)

    async def _main(self) -> None:
        # TODO Make a global settings overlay in our local path and stop mucking with user's
        #  actual global settings.
        # https://intellij-support.jetbrains.com/hc/en-us/articles/
        # 207240985-Changing-IDE-default-directories-used-for-config-plugins-and-caches-storage

        # TODO overwrite options/security.xml with KEEPASS
        await gather(
            self._create_cpp_toolchain_xml(),
            self._create_deployment_xml(),
            self._create_webservers_xml()
        )

    async def _create_cpp_toolchain_xml(self) -> None:
        root_element = _merge_elements(
            from_element=self.cpp_toolchains_xml_element,
            into_element=_get_root_element_from_path(self.cpp_toolchains_xml_path)
        )

        await exec(f'mkdir -p {self.cpp_toolchains_xml_path_base}')
        with open(self.cpp_toolchains_xml_path, 'w') as cpp_toolchains_xml_f_out:
            cpp_toolchains_xml_f_out.write(
                etree.tostring(root_element, pretty_print=True, encoding=str)
            )

    async def _create_deployment_xml(self) -> None:
        root_element = _merge_elements(
            from_element=self.deployment_xml_element,
            into_element=_get_root_element_from_path(self.deployment_xml_path)
        )

        await exec(f'mkdir -p {self.deployment_xml_path_base}')
        with open(self.deployment_xml_path, 'wb') as deployment_xml_f_out:
            deployment_xml_f_out.write(
                etree.tostring(
                    root_element, pretty_print=True, xml_declaration=True, encoding='UTF-8'
                )
            )

    async def _create_webservers_xml(self) -> None:
        root_element = _merge_elements(
            from_element=self.webservers_xml_element,
            into_element=_get_root_element_from_path(self.webservers_xml_path)
        )

        await exec(f'mkdir -p {self.webservers_xml_path_base}')
        with open(self.webservers_xml_path, 'w') as webservers_xml_f_out:
            webservers_xml_f_out.write(
                etree.tostring(root_element, pretty_print=True, encoding=str)
            )


def _get_root_element_from_path(path: str) -> Optional[_Element]:
    parser = etree.XMLParser(remove_blank_text=True)
    try:
        root_element: _Element = etree.parse(path, parser).getroot()
    except OSError:
        return None

    return root_element


def _make_key(element: _Element) -> _Key:
    return _Key(
        tag=element.tag,
        attrib=frozendict({
            k: _FIND_UUID_REGEX.sub('XXXX', v) for k, v in element.attrib.items()
        })
    )


def _merge_elements(from_element: Optional[_Element], into_element: Optional[_Element]) -> _Element:
    assert from_element is not None or into_element is not None

    if from_element is None:
        from_element = Element(into_element.tag, into_element.attrib)
    elif into_element is None:
        into_element = Element(from_element.tag, from_element.attrib)

    assert _make_key(from_element) == _make_key(into_element)

    element: _Element = Element(from_element.tag, from_element.attrib)

    from_child_element_by_key: Dict[_Key, _Element] = {}
    for from_child_element in from_element:
        from_child_element_by_key[_make_key(from_child_element)] = from_child_element

    into_child_element_by_key: Dict[_Key, _Element] = {}
    for into_child_element in into_element:
        into_child_element_by_key[_make_key(into_child_element)] = into_child_element

    for keys in (
            from_child_element_by_key.keys() - into_child_element_by_key.keys(),
            from_child_element_by_key.keys() & into_child_element_by_key.keys(),
            into_child_element_by_key.keys() - from_child_element_by_key.keys()
    ):
        for key in keys:
            element.append(
                _merge_elements(
                    from_element=from_child_element_by_key.get(key),
                    into_element=into_child_element_by_key.get(key)
                )
            )

    return element
