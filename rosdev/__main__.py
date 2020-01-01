#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK

from typing import List, Optional


def main(args: Optional[List[str]] = None) -> int:
    from rosdev.util.parser import get_options
    options = get_options(args)

    from rosdev.util.path import Path
    Path.set_store(Path.rosdev() / options.release / options.architecture)

    from atools import memoize
    memoize.set_default_db_path(Path.store() / 'db')

    import logging
    import sys
    log = logging.getLogger(__package__)
    stream_handler = logging.StreamHandler(sys.stdout)
    # noinspection PyProtectedMember,PyUnresolvedReferences
    stream_handler.setLevel(logging._nameToLevel[options.log_level])
    # noinspection PyProtectedMember,PyUnresolvedReferences
    log.setLevel(logging._nameToLevel[options.log_level])
    log.addHandler(stream_handler)

    from importlib import import_module
    handler = getattr(import_module(options.handler_module), options.handler_class)

    async def run_handler():
        await handler

    import asyncio
    # FIXME add signal handler to cancel coroutines
    asyncio.run(run_handler())

    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main(sys.argv[1:]))
