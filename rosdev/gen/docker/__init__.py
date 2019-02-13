#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK

from argparse import ArgumentParser
from typing import List, Optional


def argument_parser(
        parser: Optional[ArgumentParser] = None, parents: Optional[List[ArgumentParser]] = None
) -> ArgumentParser:
    parser = parser if parser is not None else ArgumentParser()
    parents = parents if parents is not None else []

    parent_parser = ArgumentParser(add_help=False)
    parents += [parent_parser]

    default = 'amd64'
    parent_parser.add_argument(
        '--architecture', '-a',
        default=[default],
        nargs='+',
        choices=sorted({default, 'arm32v7', 'arm64v8'}),
        help=f'List of architectures to build. Default: {default}',
    )

    default = 'crystal'
    parent_parser.add_argument(
        '--release', '-r',
        default=[default],
        nargs='+',
        choices=sorted({'ardent', 'bionic', default, 'kinetic', 'melodic'}),
        help=f'List of ROS releases to build. Default: {default}',
    )

    sub_parser = parser.add_subparsers(required=True)

    from .image import argument_parser
    argument_parser(sub_parser.add_parser(name='image', parents=parents), parents=parents)

    from .volume import argument_parser
    argument_parser(sub_parser.add_parser(name='volume', parents=parents), parents=parents)

    from .container import argument_parser
    argument_parser(sub_parser.add_parser(name='container', parents=parents), parents=parents)

    return parser


if __name__ == '__main__':
    import sys
    from rosdev import main
    sys.exit(main(parser=argument_parser()))
