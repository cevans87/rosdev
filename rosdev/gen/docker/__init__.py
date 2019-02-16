#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK

from argparse import ArgumentParser
from typing import List, Optional

from rosdev.parser import (
    architecture_parser,
    architectures_parser,
    interactive_parser,
    ports_parser,
    release_parser,
    releases_parser,
)


def argument_parser(
        parser: Optional[ArgumentParser] = None, parents: Optional[List[ArgumentParser]] = None
) -> ArgumentParser:
    parser = parser if parser is not None else ArgumentParser()
    parents = parents if parents is not None else []

    sub_parser = parser.add_subparsers(required=True)

    images_parents = parents + [architectures_parser, releases_parser]
    from .images import argument_parser
    argument_parser(sub_parser.add_parser(
        name='image', parents=images_parents), parents=images_parents)

    volume_parents = parents + [architecture_parser, release_parser]
    from .volume import argument_parser
    argument_parser(sub_parser.add_parser(
        name='volume', parents=volume_parents), parents=volume_parents)

    container_parents = parents + [
        architecture_parser, interactive_parser, ports_parser, release_parser]
    from .container import argument_parser
    argument_parser(sub_parser.add_parser(
        name='container', parents=container_parents), parents=container_parents)

    return parser


if __name__ == '__main__':
    import sys
    from rosdev import main
    sys.exit(main(parser=argument_parser()))
