from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from lxml import etree
from pathlib import Path
from textwrap import dedent

from rosdev.gen.docker.ssh import GenDockerSsh
from rosdev.gen.host import GenHost
from rosdev.gen.idea.home import GenIdeaHome
from rosdev.gen.idea.workspace import GenIdeaWorkspace
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.xml import get_root_element_from_path, merge_elements


log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaClionWebserversXml(Handler):

    @classmethod
    @memoize
    async def get_name(cls, options: Options) -> str:
        name = f'rosdev_remote ({await GenIdeaWorkspace.get_uuid(options)})'
        
        log.debug(f'{cls.__name__} {name = }')
        
        return name

    @classmethod
    @memoize
    async def get_path(cls, options: Options) -> Path:
        path = await GenIdeaHome.get_path(options) / 'options' / 'webServers.xml'

        log.debug(f'{cls.__name__} {path = }')

        return path

    @classmethod
    @memoize
    async def get_text(cls, options: Options) -> str:
        from_element = etree.fromstring(
            parser=etree.XMLParser(remove_blank_text=True),
            text=dedent(f'''
                <application>
                  <component name="WebServers">
                    <option name="servers">
                      <webServer
                          id="{await GenIdeaWorkspace.get_uuid(options)}"
                          name="{await cls.get_name(options)}" 
                          url="http:///">
                        <fileTransfer
                            host="localhost"
                            port="{await GenDockerSsh.get_port(options)}"
                            privateKey="$USER_HOME$/.ssh/id_rsa" 
                            accessType="SFTP"
                            keyPair="true">
                          <option
                              name="port"
                              value="{await GenDockerSsh.get_port(options)}" />
                        </fileTransfer>
                      </webServer>
                    </option>
                  </component>
                </application>
            ''').strip()
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
