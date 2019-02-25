from __future__ import annotations
from atools import memoize
from dataclasses import dataclass
from typing import Generator, Optional

from rosdev.gen.docker.container import container


@dataclass(frozen=True)
class _Build:
    architecture: str
    asan: bool
    build_num: Optional[int]
    colcon_build_args: Optional[str]
    release: str

    @property
    def command(self) -> str:
        command = 'colcon build'
        if self.colcon_build_args is not None:
            command += f' {self.colcon_build_args}'

        if self.asan:
            command += f' --cmake-args -DCMAKE_BUILD_TYPE=Debug ' \
                f'-DCMAKE_CXX_FLAGS_DEBUG="-fno-omit-frame-pointer -fsanitize=address"'

        return command

    def __await__(self) -> Generator[_Build, None, None]:
        async def __await__inner() -> None:
            return await container(
                architecture=self.architecture,
                build_num=self.build_num,
                command=self.command,
                interactive=False,
                ports=frozenset(),
                release=self.release,
            )

        return __await__inner().__await__()



@memoize
async def build(
        *,
        architecture: str,
        asan: bool,
        build_num: Optional[int],
        colcon_build_args: Optional[str],
        release: str,
) -> None:
    await _Build(
        architecture=architecture,
        asan=asan,
        build_num=build_num,
        colcon_build_args=colcon_build_args,
        release=release,
    )

