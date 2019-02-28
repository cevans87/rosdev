from atools import memoize
from dataclasses import dataclass
from typing import Optional

from rosdev.gen.docker.container import Container
from rosdev.util.handler import Handler


@dataclass(frozen=True)
class Build(Handler):
    architecture: str
    asan: bool
    build_num: Optional[int]
    colcon_build_args: Optional[str]
    fast: bool
    release: str

    @memoize
    async def _run(self) -> None:
        await Container(
            architecture=self.architecture,
            build_num=self.build_num,
            command=self.command,
            fast=self.fast,
            interactive=False,
            ports=frozenset(),
            release=self.release,
        )

    @property
    def command(self) -> str:
        command = 'colcon build'
        if self.colcon_build_args is not None:
            command += f' {self.colcon_build_args}'

        if self.asan:
            command += f' --cmake-args -DCMAKE_BUILD_TYPE=Debug ' \
                f'-DCMAKE_CXX_FLAGS_DEBUG="-fno-omit-frame-pointer -fsanitize=address"'

        return command
