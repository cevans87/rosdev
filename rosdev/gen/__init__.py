#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK

from argparse import ArgumentParser
from typing import List, Optional


def argument_parser(
        parser: Optional[ArgumentParser] = None, parents: Optional[List[ArgumentParser]] = None
) -> ArgumentParser:
    parser = parser if parser is not None else ArgumentParser()
    parents = parents if parents is not None else []

    sub_parser = parser.add_subparsers(required=True)

    from .docker import argument_parser
    argument_parser(sub_parser.add_parser(name='docker', parents=parents), parents=parents)

    from .workspace import argument_parser
    argument_parser(sub_parser.add_parser(name='workspace', parents=parents), parents=parents)

    return parser


if __name__ == '__main__':
    import sys
    from rosdev import main
    sys.exit(main(parser=argument_parser()))
