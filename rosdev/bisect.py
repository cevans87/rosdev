from __future__ import annotations
from asyncio import get_event_loop
from atools import memoize
from dataclasses import dataclass, field
from logging import getLogger
from jenkins import Jenkins
from typing import FrozenSet, Generator, Optional, Tuple

from rosdev.gen.docker.container import container
from rosdev.gen.colcon.build import build
from rosdev.gen.install import install
from rosdev.util.lookup import get_operating_system


log = getLogger(__package__)


@dataclass(frozen=True)
class _Bisect:
    architecture: str
    asan: bool
    bad_build_num: Optional[int]
    good_build_num: Optional[int]
    colcon_build_args: Optional[str]
    release: str

    builds: Tuple[int] = field(init=False)

    def __await__(self) -> Generator[_Bisect, None, None]:
        async def __await___inner() -> None:
            await self._init_builds()

            await container(
                architecture=self.architecture,
                build_num=self.bad_build_num,
                release=self.release,
                ports=frozenset(),
                interactive=False,
                command=f'colcon build'
                f'{" " + " ".join(self.colcon_build_args) if self.colcon_build_args else ""}'
            )

        return __await___inner().__await__()

    async def _init_builds(self) -> None:
        def _init_builds_inner() -> None:
            server = Jenkins('https://ci.ros2.org')
            job_info = server.get_job_info(
                f'packaging_{get_operating_system(self.architecture)}',
            )

            builds = tuple()

            object.__setattr__(self, 'builds', builds)

        await get_event_loop().run_in_executor(None, _init_builds_inner)


bisect = memoize(_Bisect)

@memoize
async def bisect(
        *,
        architecture: str,
        bad_build_num: Optional[int],
        good_build_num: Optional[int],
        colcon_build_args: Optional[str],
        release: str,
) -> None:
    await _Bisect(
        architecture=architecture,
        bad_build_num=bad_build_num,
        good_build_num=good_build_num,
        colcon_build_args=colcon_build_args,
        release=release,
    )

