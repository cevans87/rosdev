from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from lxml import etree
from textwrap import dedent

from rosdev.gen.host import GenHost
from rosdev.gen.idea.workspace import GenIdeaWorkspace
from rosdev.gen.workspace import GenWorkspace
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.path import Path


log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaClionIml(Handler):

    @classmethod
    @memoize
    async def get_bytes(cls, options: Options) -> bytes:
        element = etree.fromstring(
            parser=etree.XMLParser(remove_blank_text=True),
            text=dedent(f'''
                <module classpath="CMake" type="CPP_MODULE" version="4" />
            ''').lstrip()
        )
        # noinspection PyShadowingBuiltins
        bytes = etree.tostring(element, pretty_print=True, xml_declaration=True, encoding='UTF-8')

        log.debug(f'{cls.__name__} {bytes = }')

        return bytes

    @classmethod
    @memoize
    async def get_path(cls, options: Options) -> Path:
        path = (
                await GenIdeaWorkspace.get_path(options) /
                f'{(await GenWorkspace.get_path(options)).parts[-1]}.iml'
        )

        log.debug(f'{cls.__name__} {path}')

        return path

    @classmethod
    async def main(cls, options: Options) -> None:
        GenHost.write_bytes(
            data=await cls.get_bytes(options),
            options=options,
            path=await cls.get_path(options),
        )
