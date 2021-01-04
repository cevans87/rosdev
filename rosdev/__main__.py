#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK


def main() -> int:
    from rosdev.node import Node
    from rosdev.parser import Parser
    try:
        node = Parser().to_node()
    except Node.Exception as e:
        print(f'\n{e}', file=sys.stderr)
        exit(1)
        raise
        
    import logging
    logging.basicConfig(level=node.options.log_level.upper())
    
    import asyncio
    try:
        asyncio.run(node.cli_entrypoint())
    except Node.Exception as e:
        print(f'\n{e}', file=sys.stderr)
        exit(1)
        raise

    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())
