from asyncio import get_event_loop
from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from jenkins import Jenkins
from typing import List, Optional, Tuple

from rosdev.gen.docker.container import Container
from rosdev.gen.colcon.build import Build
from rosdev.gen.install import Install
from rosdev.util.handler import Handler
from rosdev.util.lookup import get_operating_system
from rosdev.util.subprocess import shell


log = getLogger(__package__)


@memoize
@dataclass(frozen=True)
class Bisect(Handler):
    architecture: str
    asan: bool
    bad_build: str
    bad_build_num: Optional[int]
    colcon_build_args: Optional[str]
    command: str
    fast: bool
    good_build: str
    good_build_num: Optional[int]
    release: str

    @memoize
    async def _run(self) -> None:
        build_nums = await self._get_build_nums()

        i = 0
        while len(build_nums) > 2:
            log.info(f'testing midpoint betwee {build_nums[0]} and {build_nums[-1]}')
            test_build_num = build_nums[len(build_nums) // 2]
            log.info(f'checking if build {test_build_num} is testable...')
            if not await self._get_build_num_succeeded(test_build_num):
                log.info(f'skipping untestable build {test_build_num}')
                build_nums = \
                    build_nums[:len(build_nums) // 2] + build_nums[(len(build_nums) // 2) + 1:]
                continue

            log.info(f'testing build {test_build_num}')

            await shell('rm -rf build install log')

            await Install(
                architecture=self.architecture,
                build_num=test_build_num,
                fast=self.fast,
                release=self.release,
            )

            await Build(
                architecture=self.architecture,
                asan=self.asan,
                build_num=test_build_num,
                colcon_build_args=self.colcon_build_args,
                fast=self.fast,
                release=self.release,
            )

            test_exit_code = await Container(
                architecture=self.architecture,
                command=self.command,
                build_num=test_build_num,
                fast=self.fast,
                interactive=False,
                ports=frozenset(),
                release=self.release,
            ).exit_code()

            if test_exit_code == 0:
                log.info(f'build {test_build_num} is good')
                build_nums = build_nums[len(build_nums) // 2:]
            else:
                log.info(f'build {test_build_num} is bad')
                build_nums = build_nums[:(len(build_nums) // 2) + 1]

            i += 1

        good_build_num, bad_build_num = build_nums

        log.info(
            f'bisect found good build num: {good_build_num}, '
            f'bad build num: {bad_build_num}, after {i} iterations'
        )

    @memoize
    async def _get_build_nums(self) -> Tuple[int]:
        def _get_builds_nums_inner() -> Tuple[int]:
            server = Jenkins('https://ci.ros2.org')
            job_info = server.get_job_info(
                f'packaging_{get_operating_system(self.architecture)}',
            )

            build_nums: List[int] = []
            for build_info in job_info['builds']:
                build_num = build_info['number']
                if (self.good_build_num is not None) and (build_num < self.good_build_num):
                    continue
                if (self.bad_build_num is not None) and (build_num > self.bad_build_num):
                    continue
                build_nums.append(build_num)

            return tuple(sorted(build_nums))

        return await get_event_loop().run_in_executor(None, _get_builds_nums_inner)

    @memoize
    async def _get_build_num_succeeded(self, build_num: int) -> bool:
        def _get_build_num_succeeded_inner() -> bool:
            server = Jenkins('https://ci.ros2.org')
            build_info = server.get_build_info(
                f'packaging_{get_operating_system(self.architecture)}', build_num)

            return build_info['result'] == 'SUCCESS'

        return await get_event_loop().run_in_executor(None, _get_build_num_succeeded_inner)
