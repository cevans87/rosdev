#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK


def main() -> int:
    from rosdev.util.options import Options
    options = Options.of_args()

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

    if options.reset_caches:
        # FIXME even though our module is already loaded, some memoize decorators (like those used
        #  on inner functions) may not be evaluated yet. Such decorators will not be reset.
        from rosdev.util.atools import memoize
        memoize.reset_all()

    import asyncio
    # FIXME add signal handler to cancel coroutines
    asyncio.run(handler.run(options))

    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())
