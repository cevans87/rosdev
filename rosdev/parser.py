from __future__ import annotations
from argcomplete import autocomplete, FilesCompleter
from argparse import ArgumentParser, REMAINDER
from dataclasses import dataclass, field
from importlib import import_module
from importlib.util import find_spec
from inspect import getdoc 
from pkgutil import iter_modules
import platform
import sys
from typing import FrozenSet, Sequence, Set, Tuple

import rosdev
from rosdev.node import Node
from rosdev.options import Options
from rosdev.path import Path
from rosdev.third_party.atools import memoize_db


@dataclass(frozen=True)
class _ParserStructure:
    command_parts: Tuple[str, ...]
    doc: str
    sub_parser_structures: FrozenSet[_ParserStructure] = frozenset()

    def get_command_class(self) -> str:
        return ''.join([
            command_part.capitalize()
            for command_part in self.command_parts
            for command_part in command_part.split('_')
            if command_part
        ])

    def get_command_module(self) -> str:
        return '.'.join(['rosdev.cmd', *self.command_parts])

    @classmethod
    def of_command_parts(cls, command_parts: Tuple[str, ...]) -> _ParserStructure:
        module_name = '.'.join(['rosdev.cmd', *command_parts])
        spec = find_spec(module_name)
        module = import_module(module_name)
        if spec.submodule_search_locations is None:
            command_class = ''.join([
                command_part.capitalize()
                for command_part in command_parts
                for command_part in command_part.split('_')
                if command_part
            ])
            doc = getdoc(module.__dict__[command_class]) or ''
            parser_structure = _ParserStructure(command_parts=command_parts, doc=doc)
        else:
            doc = getdoc(module)
            sub_parser_structures: Set[_ParserStructure] = set()
            for module in iter_modules(spec.submodule_search_locations):
                if not (sub_command := module.name).startswith('_'):
                    sub_parser_structures.add(
                        cls.of_command_parts(tuple([*command_parts, sub_command]))
                    )
            parser_structure = _ParserStructure(
                command_parts=command_parts,
                doc=doc,
                sub_parser_structures=frozenset(sub_parser_structures)
            )

        return parser_structure
    

@dataclass(frozen=True)
class Parser:
    args: Sequence[str] = field(default=tuple(sys.argv[1:]))

    @staticmethod
    @memoize_db(size=1, keygen=lambda: Path(rosdev.__file__).parent.stat().st_mtime)
    def get_parser_structure() -> _ParserStructure:
        return _ParserStructure.of_command_parts(tuple())

    @staticmethod
    def get_parser() -> ArgumentParser:
        def add_flags(parser: ArgumentParser) -> None:
            parser.add_argument(
                'args',
                nargs=REMAINDER,
                help=f'Args to be forwarded on to docker run, if applicable.'
            )
            architecture_default = {
                'x86_64': Options.Architecture.amd64,
                'aarch64': Options.Architecture.arm64v8,
            }[platform.machine()]
            parser.add_argument(
                '--architecture',
                default=architecture_default,
                type=Options.Architecture,
                choices=list(Options.Architecture.__members__.keys()),
                help=f'Architecture to build upon. Default: "{architecture_default}"'
            )
            log_level_default = Options.LogLevel.info
            parser.add_argument(
                '--log-level',
                default=log_level_default,
                choices=list(Options.LogLevel.__members__.keys()),
                type=Options.LogLevel,
                help=f'Log level. Default: "{log_level_default}"'
            )
            mounts_default = []
            parser.add_argument(
                '--mounts',
                default=[],
                nargs='*',
                help=(
                    f'<host_path>:<container_path> mounts to be created (or if already created,'
                    f' resumed) during `docker run`. Paths may be specified as relative paths.'
                    f' <host_path> relative paths are relative to the host\'s current working'
                    f' directory. <container_path> relative paths are relative to the Docker'
                    f' container\'s WORKDIR (AKA the build dir). Default:'
                    f' "{" ".join(mounts_default)}"'
                )
            )
            ports_default = []
            parser.add_argument(
                '--ports',
                default=ports_default,
                nargs='*',
                type=int,
                help=f'Ports to expose in underlying docker container. Default: "{ports_default}"'
            )
            release_default = Options.Release.foxy
            parser.add_argument(
                '--release',
                default=release_default,
                choices=Options.Release.__members__.keys(),
                type=Options.Release,
                help=f'ROS release. Default: "{release_default}"'
            )

        def inner(parser: ArgumentParser, parser_structure: _ParserStructure) -> ArgumentParser:
            if not parser_structure.sub_parser_structures:
                add_flags(parser)
                parser.set_defaults(
                    command_class=parser_structure.get_command_class(),
                    command_module=parser_structure.get_command_module(),
                )
            else:
                sub_parsers = parser.add_subparsers()
                for sub_parser_structure in parser_structure.sub_parser_structures:
                    sub_parser = sub_parsers.add_parser(sub_parser_structure.command_parts[-1])
                    inner(sub_parser, sub_parser_structure)
            parser.description = parser_structure.doc

            return parser

        return inner(ArgumentParser(prog='rosdev'), parser_structure=Parser.get_parser_structure())

    def to_node(self) -> Node:
        """Return Node constructed by parsing args as if from sys.argv."""

        parser = self.get_parser()
        autocomplete(parser, default_completer=FilesCompleter(allowednames='', directories=False))

        parsed_args = parser.parse_args(self.args).__dict__
        if not parsed_args:
            Node.get_parser_structure.memoize.remove()
            parser.parse_args([*self.args, '--help'])
            sys.exit(1)

        if parsed_args['args'] and parsed_args['args'][0] == '--':
            parsed_args['args'] = parsed_args['args'][1:]
        parsed_args['args'] = ' '.join(parsed_args['args'])
        parsed_args['ports'] = frozenset(str(port) for port in parsed_args['ports'])

        mounts = []
        for mount in parsed_args['mounts']:
            host_path, container_path = mount.split(':', 1)
            host_path, container_path = Path(host_path), Path(container_path)
            if not host_path.is_absolute():
                host_path = host_path.absolute()
            if not container_path.is_absolute():
                container_path = container_path.absolute().image_build()
            mounts.append(Options.Mount(container_path=container_path, host_path=host_path))
        parsed_args['mounts'] = frozenset(mounts)

        command_class = parsed_args.pop('command_class')
        command_module = parsed_args.pop('command_module')

        options = Options(**parsed_args)
        
        try:
            node = getattr(import_module(command_module), command_class)(options)
        except AttributeError:
            print(
                f'Invalid command "{command_class.replace(".", " ")}"\n{parser.format_usage()}',
                file=sys.stderr
            )
            sys.exit(1)

        return node
