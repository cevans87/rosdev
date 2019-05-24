from atools import memoize
from dataclasses import dataclass
from frozendict import frozendict
from logging import getLogger
from lxml import etree
# noinspection PyProtectedMember
from lxml.etree import _Element
from pathlib import Path
from textwrap import dedent
from typing import Mapping

from rosdev.gen.clion.config import Config as ClionConfig
from rosdev.gen.install import Install
from rosdev.util.subprocess import get_shell_lines
from rosdev.util.handler import Handler
from rosdev.util.xml import get_root_element_from_path, merge_elements
from rosdev.util.subprocess import exec


log = getLogger(__name__)


@memoize
@dataclass(frozen=True)
class WorkspaceXml(Handler):

    @property
    def local_path_base(self) -> str:
        return ClionConfig(self.options).local_path

    @property
    def local_path(self) -> str:
        return f'{self.local_path_base}/workspace.xml'

    async def get_element(self) -> _Element:
        return etree.fromstring(
            parser=etree.XMLParser(remove_blank_text=True),
            text=dedent(f'''
                <project version="4">
                  <component name="CMakeSettings">
                    <configurations>
                      <configuration
                          PROFILE_NAME="{self.options.build_type}"
                          CONFIG_NAME="{self.options.build_type}">
                        <ADDITIONAL_GENERATION_ENVIRONMENT>
                          <envs>
                            {' '.join([
                                f'<env name="{k}" value="{v}" />' 
                                for k, v in (await self.get_environment()).items()
                            ])}
                          </envs>
                        </ADDITIONAL_GENERATION_ENVIRONMENT>
                      </configuration>
                    </configurations>
                  </component>
                </project>
            ''').strip()
        )

    @memoize
    async def get_environment(self) -> Mapping[str, str]:
        await Install(self.options)

        command = f'bash -c \''
        if self.options.global_setup is not None:
            command += f'source {Path(self.options.global_setup).expanduser().absolute()} && '
        if self.options.local_setup is not None:
            command += f'source {Path(self.options.local_setup).expanduser().absolute()} && '
        command += 'env\''

        lines = await get_shell_lines(command)
        environment = {}
        for line in lines:
            if line:
                k, v = line.split('=', 1)
                if k in {
                    'AMENT_PREFIX_PATH',
                    'CMAKE_PREFIX_PATH',
                    'COLCON_PREFIX_PATH',
                    'LD_LIBRARY_PATH',
                    'PATH',
                    'PYTHONPATH',
                    'ROS_DISTRO',
                    'ROS_PYTHON_VERSION',
                    'ROS_VERSION',
                }:
                    environment[k] = v
        # FIXME LD_PRELOAD should happen at runtime, not compile time
        #if self.options.sanitizer is not None:
        #    if self.options.sanitizer == 'asan':
        #        environment['LD_PRELOAD'] = '/usr/lib/x86_64-linux-gnu/libasan.so.4'
        #    else:
        #        environment['LD_PRELOAD'] = (
        #            f'/usr/lib/x86_64-linux-gnu/lib{self.options.sanitizer}.so.0'
        #        )

        return frozendict(environment)

    @memoize
    async def _main(self) -> None:
        root_element = merge_elements(
            from_element=await self.get_element(),
            into_element=get_root_element_from_path(self.local_path)
        )

        await exec(f'mkdir -p {self.local_path_base}')
        with open(self.local_path, 'wb') as workspace_xml_f_out:
            workspace_xml_f_out.write(
                etree.tostring(
                    root_element, pretty_print=True, xml_declaration=True, encoding='UTF-8'
                )
            )
