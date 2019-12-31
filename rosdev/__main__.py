#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK

from importlib import import_module
import logging
from typing import List, Optional
from rosdev.util.parser import get_options

log = logging.getLogger(__package__)


def main(args: Optional[List[str]] = None) -> int:
    options = get_options()

    rosdev_handler_module = args.__dict__.pop('rosdev_handler_module')
    rosdev_handler_class = args.__dict__.pop('rosdev_handler_class')

    from rosdev.util.options import Options

    parsed_options: Options = replace(options, **args.__dict__)

    from atools import memoize
    from rosdev.util.path import Path
    Path.set_store(Path.rosdev() / parsed_options.release / parsed_options.architecture)
    memoize.set_default_db_path(Path.store() / 'db')

    handler = getattr(import_module(rosdev_handler_module), rosdev_handler_class)

    return handler.run(parsed_options), parsed_options

    import sys
    stream_handler = logging.StreamHandler(sys.stdout)
    # noinspection PyProtectedMember,PyUnresolvedReferences
    stream_handler.setLevel(logging._nameToLevel[options.log_level])
    # noinspection PyProtectedMember,PyUnresolvedReferences
    log.setLevel(logging._nameToLevel[options.log_level])
    log.addHandler(stream_handler)

    async def run_handler():
        await handler

    import asyncio
    # FIXME add signal handler to cancel coroutines
    asyncio.run(run_handler())

    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main(sys.argv[1:]))
