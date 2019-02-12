#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK

from argparse import ArgumentParser
from typing import List, Optional

try:
    from argcomplete import autocomplete
except ImportError:
    def autocomplete(_parser: ArgumentParser): ...


def argument_parser(
        parser: Optional[ArgumentParser] = None, parents: Optional[List[ArgumentParser]] = None
) -> ArgumentParser:
    parser = parser if parser is not None else ArgumentParser()
    parents = parents if parents is not None else []

    parser.add_argument('-a', '--attach', action='store_true')

    def func(**kwargs) -> int:
        import asyncio
        from . import container

        asyncio.run(container.gen_docker_containers(**kwargs))

        return 0

    parser.set_defaults(func=func)

    return parser


if __name__ == '__main__':
    import sys
    from rosdev import main
    sys.exit(main(parser=argument_parser()))
