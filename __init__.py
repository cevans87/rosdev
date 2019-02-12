#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK

from argparse import ArgumentParser
from typing import List, Optional

try:
    from argcomplete import autocomplete
except ImportError:
    def autocomplete(_parser: ArgumentParser): ...


def _rosdev(args: List[str]) -> int:
    from . import rosdev

    return rosdev.main(args)


def argument_parser(parser: ArgumentParser = None) -> ArgumentParser:
    parser = parser or ArgumentParser(add_help=False)
    sub_parser = parser.add_subparsers(required=True)

    rosdev_parser = sub_parser.add_parser(name='rosdev')
    rosdev_parser.set_defaults(func=_rosdev)

    return parser


def main(args: Optional[List[str]] = None) -> int:
    import sys

    args = args if args is not None else sys.argv[1:]

    parser = argument_parser()
    autocomplete(parser)
    try:
        known_args, unknown_args = parser.parse_known_args(args)
    except Exception:
        parser.print_usage()
    else:
        return known_args.func(unknown_args)


if __name__ == '__main__':
    import sys
    sys.exit(main())
