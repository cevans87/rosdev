from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from pyperclip import copy
from textwrap import dedent

from rosdev.util.handler import Handler
from rosdev.util.lookup import get_environ


log = getLogger(__name__)


@memoize
@dataclass(frozen=True)
class Cmake(Handler):

    @memoize
    async def _main(self) -> None:
        copy('\n'.join(f'{k}={v}' for k, v in (await get_environ()).items()))
        log.info('CMake environment variables copied to clipboard')
        log.info(dedent('''
            CMake Setup (Do this once for your current project and when CMakeLists.txt changes)
                1. Go to Settings | Build, Execution, Deployment > CMake
                2. Use an existing CMake profile or create a new one
                3. Build type: probably "Debug"
                4. Toolchain: The toolchain you created with "rosdev gen clion toolchain"
                5. CMake options: leave blank in most cases
                6. Environment: press "shift+enter" or click icon to bring up sub menu
                    a. Click the "paste" icon
                7. Generation path: set to "build"
        '''))
