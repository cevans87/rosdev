#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK

from argparse import ArgumentParser
import logging
from typing import List, Optional
from .parser import (
    architecture_parser,
    log_level_parser,
    ports_parser,
    release_parser,
)

try:
    from argcomplete import autocomplete
except ImportError:
    def autocomplete(_parser: ArgumentParser): ...


log = logging.getLogger(__name__)


def argument_parser(
        parser: Optional[ArgumentParser] = None, parents: Optional[List[ArgumentParser]] = None
) -> ArgumentParser:
    parser = parser if parser is not None else ArgumentParser()
    parents = parents if parents is not None else []

    sub_parser = parser.add_subparsers(required=True)

    bash_parents = parents + [
        architecture_parser, log_level_parser, release_parser, ports_parser]
    from .bash import argument_parser
    argument_parser(sub_parser.add_parser(
        name='bash', parents=bash_parents), parents=bash_parents)

    clion_parents = parents + [architecture_parser, log_level_parser, release_parser]
    from .clion import argument_parser
    argument_parser(sub_parser.add_parser(
        name='clion', parents=clion_parents), parents=clion_parents)

    gdb_parents = parents + [architecture_parser, log_level_parser, release_parser]
    from .gdb import argument_parser
    argument_parser(sub_parser.add_parser(
        name='gdb', parents=gdb_parents), parents=gdb_parents)

    gen_parents = parents + [log_level_parser]
    from .gen import argument_parser
    argument_parser(sub_parser.add_parser(
        name='gen', parents=gen_parents), parents=gen_parents)

    return parser


def main(args: Optional[List[str]] = None, parser: Optional[ArgumentParser] = None) -> int:
    import sys

    args = args if args is not None else sys.argv[1:]

    parser = parser if parser is not None else argument_parser()
    autocomplete(parser)

    handler = logging.StreamHandler(sys.stdout)
    log.addHandler(handler)

    try:
        args = parser.parse_args(args)
    except Exception:
        parser.print_usage()
        return 1
    else:
        log.setLevel(args.log_level)
        handler.setLevel(args.log_level)
        func = args.func
        del args.__dict__['func']
        del args.__dict__['log_level']
        return func(**args.__dict__)


if __name__ == '__main__':
    import sys
    from rosdev import main
    sys.exit(main(parser=argument_parser()))
