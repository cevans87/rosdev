#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK

from argparse import ArgumentParser
import logging
from typing import List, Optional
from .parser import rosdev_parser

try:
    from argcomplete import autocomplete
except ImportError:
    def autocomplete(_parser: ArgumentParser): ...


log = logging.getLogger(__name__)


def main(args: Optional[List[str]] = None, parser: Optional[ArgumentParser] = None) -> int:
    import sys

    args = args if args is not None else sys.argv[1:]

    parser = parser if parser is not None else rosdev_parser
    autocomplete(parser)

    try:
        args = parser.parse_args(args)
    except Exception:
        parser.print_usage()
        return 1

    log.setLevel(args.log_level)
    stream_handler = logging.StreamHandler(sys.stdout)
    stream_handler.setLevel(args.log_level)
    log.addHandler(stream_handler)

    handler = args.get_handler()
    del args.__dict__['get_handler']
    del args.__dict__['log_level']

    for k, v in args.__dict__.items():
        if isinstance(v, list):
            setattr(args, k, frozenset(v))

    import asyncio
    return asyncio.run(handler(**args.__dict__))


if __name__ == '__main__':
    import sys
    from . import main
    sys.exit(main(parser=rosdev_parser))
