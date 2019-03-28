from atools import memoize
from dataclasses import dataclass
from lxml.etree import Element, fromstring, SubElement, tostring, XMLParser
import os

from rosdev.util.handler import Handler
from rosdev.util.lookup import get_environ
from rosdev.util.subprocess import exec


@memoize
@dataclass(frozen=True)
class Workspace(Handler):

    @memoize
    async def _main(self) -> None:
        path_base = f'{os.getcwd()}/.idea'
        path = f'{path_base}/workspace.xml'
        try:
            with open(path, 'r') as workspace_xml_f_in:
                root = fromstring(
                    workspace_xml_f_in.read().encode(),
                    XMLParser(remove_blank_text=True),
                )
        except FileNotFoundError:
            root = Element('project', {'version': '4'})

        # TODO add Release-rosdev Release-rosdev configurations
        parent = root
        child = parent.find('./component[@name="CMakeSettings"]')
        if child is None:
            child = SubElement(parent, 'component', {'name': 'CMakeSettings'})
            parent[:] = sorted(parent, key=lambda element: element.get('name'))

        parent = child
        child = parent.find('./configurations')
        if child is None:
            child = SubElement(parent, 'configurations', {})

        parent = child
        child = parent.find(
            './configuration[@PROFILE_NAME="Release-rosdev"][@CONFIG_NAME="Release"]')
        if child is None:
            child = SubElement(
                parent,
                'configuration',
                {'PROFILE_NAME': 'Release-rosdev', 'CONFIG_NAME': 'Release'}
            )
            parent[:] = sorted(parent, key=lambda element: element.attrib['PROFILE_NAME'])

        parent = child
        child = parent.find('./ADDITIONAL_GENERATION_ENVIRONMENT')
        if child is None:
            child = SubElement(parent, 'ADDITIONAL_GENERATION_ENVIRONMENT', {})

        parent = child
        child = parent.find('./envs')
        if child is None:
            child = SubElement(parent, 'envs', {})

        parent = child
        for name, value in (await get_environ()).items():
            child = parent.find(f'./env[@name="{name}"]')
            if child is not None:
                parent.remove(child)
            SubElement(parent, 'env', {'name': name, 'value': value})
        else:
            parent[:] = sorted(parent, key=lambda element: element.attrib['name'])

        await exec(f'mkdir -p {path_base}')
        with open(path, 'wb') as workspace_xml_f_out:
            workspace_xml_f_out.write(
                tostring(
                    root,
                    encoding='utf-8',
                    method='xml',
                    pretty_print=True,
                    xml_declaration=True,
                )
            )
