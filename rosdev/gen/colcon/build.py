from atools import memoize
from dataclasses import dataclass
from typing import Optional

from rosdev.gen.docker.container import Container
from rosdev.util.handler import Handler


@memoize
@dataclass(frozen=True)
class Build(Handler):
    architecture: str
    asan: bool
    build_num: Optional[int]
    clean: bool
    colcon_build_args: Optional[str]
    debug: bool
    fast: bool
    release: str

    @property
    def command(self) -> str:
        parts = [f'colcon build']
        if self.colcon_build_args is not None:
            parts.append(self.colcon_build_args)

        if self.asan or self.debug:
            parts.append(f'--cmake-args -DCMAKE_BUILD_TYPE=Debug')
            if self.asan:
                parts.append(
                    f'-DCMAKE_CXX_FLAGS_DEBUG="-fno-omit-frame-pointer -fsanitize=address"')

        return ' '.join(parts)

    @memoize
    async def exit_code(self) -> int:
        return await Container(
            architecture=self.architecture,
            build_num=self.build_num,
            clean=self.clean,
            command=self.command,
            fast=self.fast,
            interactive=False,
            ports=frozenset(),
            release=self.release,
        ).exit_code()

    @memoize
    async def must_succeed(self) -> None:
        if await self.exit_code() != 0:
            raise Exception('Build failed.')

    @memoize
    async def _run(self) -> None:
        await self.exit_code()
