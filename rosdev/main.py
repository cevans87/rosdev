#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK

from argparse import ArgumentParser
import logging
from typing import List, Optional
from rosdev.parser import rosdev_parser

try:
    from argcomplete import autocomplete
except ImportError:
    def autocomplete(_parser: ArgumentParser): ...


log = logging.getLogger(__package__)


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

    stream_handler = logging.StreamHandler(sys.stdout)
    stream_handler.setLevel(logging._nameToLevel[args.log_level])
    log.setLevel(logging._nameToLevel[args.log_level])
    log.addHandler(stream_handler)

    handler = args.get_handler()
    del args.__dict__['get_handler']
    del args.__dict__['log_level']

    for k, v in args.__dict__.items():
        if isinstance(v, list):
            setattr(args, k, frozenset(v))

    async def run_handler():
        await handler(**args.__dict__)()

    import asyncio
    # FIXME add signal handler to cancel coroutines
    asyncio.run(run_handler())

    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main(parser=rosdev_parser))
