#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK

from argparse import ArgumentParser, REMAINDER
from typing import List, Optional


def argument_parser(
        parser: Optional[ArgumentParser] = None, parents: Optional[List[ArgumentParser]] = None
) -> ArgumentParser:
    parser = parser if parser is not None else ArgumentParser()
    parents = parents if parents is not None else []

    parser.add_argument('binary')
    parser.add_argument('remainder', nargs=REMAINDER)
    parser.add_argument('--port', '-p', type=int, default=1337)

    def func(remainder: List[str], **kwargs) -> int:
        import asyncio
        from .gdbserver import gdbserver

        asyncio.run(gdbserver(remainder=tuple(remainder), **kwargs))

        return 0

    parser.set_defaults(func=func)

    return parser


if __name__ == '__main__':
    import sys
    from rosdev import main
    sys.exit(main(parser=argument_parser()))
