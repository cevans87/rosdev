from asyncio import get_event_loop, Lock
from atools import memoize
from dataclasses import dataclass, field
from jenkins import Jenkins as _ExternalJenkins
import re
from typing import List, Optional, Tuple

from rosdev.util.lookup import get_build_num, get_operating_system
from rosdev.util.options import Options

@memoize
@dataclass(frozen=True)
class _Jenkins:
    _external_jenkins: _ExternalJenkins = field(
        init=False, default_factory=lambda: _ExternalJenkins('https://ci.ros2.org')
    )

    def get_job_name(self, architecture: str) -> str:
        return f'packaging_{get_operating_system(architecture)}'

    @memoize
    async def get_build_num(self, architecture: str, release: str) -> int:
        try:
            return {
                'amd64': {
                    'crystal': 1289,
                },
                'arm32v7': {

                },
                'arm64v8': {
                    'crystal': 651,
                },
            }[architecture][release]
        except KeyError:
            pass

        def get_build_num_inner() -> int:
            return self._external_jenkins.get_job_info(
                name=self.get_job_name(architecture=architecture),
                depth=1,
                fetch_all_builds=False,
            )['lastSuccessfulBuild']['number']

        return await get_event_loop().run_in_executor(None, get_build_num_inner)

    async def get_build_console_output(
            self, architecture: str, build_num: Optional[int], release: str
    ) -> Tuple[str]:
        if build_num is None:
            build_num = await self.get_build_num(architecture=architecture, release=release)

        def get_build_console_output_inner() -> Tuple[str]:
            return tuple(self._external_jenkins.get_build_console_output(
                f'packaging_{get_operating_system(architecture)}', build_num
            ).splitlines())

        return await get_event_loop().run_in_executor(None, get_build_console_output_inner)


@memoize
@dataclass(frozen=True)
class _JenkinsContext:
    _lock: Lock = field(init=False, default_factory=Lock)
    _jenkins: _Jenkins = field(init=False, default_factory=_Jenkins)

    async def __aenter__(self) -> _Jenkins:
        await self._lock.acquire()

        return self._jenkins

    async def __aexit__(self, exc_type, exc_val, exc_tb) -> None:
        self._lock.release()


async def get_ros2_repos(architecture: str, build_num: Optional[int], release: str) -> str:
    async with _JenkinsContext() as jenkins:
        lines = await jenkins.get_build_console_output(
            architecture=architecture, build_num=build_num, release=release
        )

    remaining_lines = iter(lines)
    for line in remaining_lines:
        if re.match(r'^# BEGIN SUBSECTION: vcs export --exact$', line) is not None:
            break
    for line in remaining_lines:
        if re.match(r'^repositories:$', line) is not None:
            break
    ros2_repos_lines: List[str] = ['repositories:']
    for line in remaining_lines:
        if re.match(r'^# END SUBSECTION$', line) is not None:
            break
        elif re.match(r'\s+\S+:.*$', line) is not None:
            ros2_repos_lines.append(line)

    return '\n'.join(ros2_repos_lines)
