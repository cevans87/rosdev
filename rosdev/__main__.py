#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK

import logging
from typing import List, Optional
from rosdev.util.parser import get_handler_and_options

log = logging.getLogger(__package__)


def main(args: Optional[List[str]] = None) -> int:
    handler, options = get_handler_and_options(args)

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
