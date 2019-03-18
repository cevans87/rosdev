from asyncio import get_event_loop
from atools import memoize
from logging import getLogger
from jenkins import Jenkins
from typing import Dict, List, Tuple

from rosdev.gen.docker.container import Container
from rosdev.gen.colcon.build import Build
from rosdev.gen.install import Install
from rosdev.util.handler import Handler
from rosdev.util.lookup import get_build_num, get_operating_system
from rosdev.util.subprocess import shell


log = getLogger(__package__)


@memoize
class Bisect(Handler):

    @memoize
    async def _main(self) -> None:
        build_nums = await self._get_build_nums()

        i = 0
        while len(build_nums) > 2:
            log.info(f'testing midpoint between {build_nums[0]} and {build_nums[-1]}')
            test_build_num = build_nums[len(build_nums) // 2]
            log.info(f'checking if build {test_build_num} is testable...')
            if not await self._get_build_num_succeeded(test_build_num):
                log.info(f'skipping untestable build {test_build_num}')
                build_nums = \
                    build_nums[:len(build_nums) // 2] + build_nums[(len(build_nums) // 2) + 1:]
                continue

            log.info(f'testing build {test_build_num}')

            await shell('rm -rf build install log')

            await Install(self.options(build_num=test_build_num))

            await Build(self.options(build_num=test_build_num)).must_succeed()

            try:
                await Container(
                    self.options(build_num=test_build_num, interactive=False)
                ).must_succeed()
            except Container.Exception:
                log.info(f'build {test_build_num} is bad')
                build_nums = build_nums[:(len(build_nums) // 2) + 1]
            else:
                log.info(f'build {test_build_num} is good')
                build_nums = build_nums[len(build_nums) // 2:]

            i += 1

        good_build_num, bad_build_num = build_nums

        log.info(
            f'bisect found good build num: {good_build_num}, '
            f'bad build num: {bad_build_num}, after {i} iterations'
        )

    @memoize
    async def _get_job_info(self) -> Dict:
        def _get_job_info_inner() -> Dict:
            return Jenkins('https://ci.ros2.org').get_job_info(
                f'packaging_{get_operating_system(self.architecture)}')

        return await get_event_loop().run_in_executor(None, _get_job_info_inner)

    @memoize
    async def _get_build_info(self, build_num: int) -> Dict:
        def _get_build_info_inner() -> Dict:
            return Jenkins('https://ci.ros2.org').get_build_info(
                f'packaging_{get_operating_system(self.architecture)}', build_num)

        return await get_event_loop().run_in_executor(None, _get_build_info_inner)

    @memoize
    async def _get_bad_build_num(self) -> int:
        if self.options.bad_build_num is not None:
            return self.options.bad_build_num
        elif self.options.bad_release != 'latest':
            return get_build_num(self.options.architecture, self.options.bad_release)
        else:
            return (await self._get_job_info())['lastSuccessfulBuild']['number']

    @memoize
    async def _get_good_build_num(self) -> int:
        if self.options.good_build_num is not None:
            return self.good_build_num
        elif self.options.good_release != 'latest':
            return get_build_num(self.options.architecture, self.options.good_release)
        else:
            return (await self._get_job_info())['lastSuccessfulBuild']['number']

    @memoize
    async def _get_build_nums(self) -> Tuple[int]:
        job_info = await self._get_job_info()

        bad_build_num = await self._get_bad_build_num()
        good_build_num = await self._get_good_build_num()

        build_nums: List[int] = []
        for build_info in job_info['builds']:
            build_num = build_info['number']
            if good_build_num <= build_num <= bad_build_num:
                build_nums.append(build_num)

        return tuple(sorted(build_nums))

    @memoize
    async def _get_build_num_succeeded(self, build_num: int) -> bool:
            return (await self._get_build_info(build_num))['result'] == 'SUCCESS'
