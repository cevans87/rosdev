from atools import memoize
from frozendict import frozendict
from typing import Mapping

from rosdev.gen.docker.container import Container
from rosdev.gen.install import Install
from rosdev.gen.rosdep.config import Config as RosdepConfig
from rosdev.util.handler import Handler


@memoize
class Build(Handler):

    @property
    def command(self) -> str:
        parts = [f'--cmake-args -DCMAKE_BUILD_TYPE={self.options.build_type}']

        if self.options.sanitizer is not None:
            flags = {
                'asan': f'-fsanitize=address',
                'lsan': f'-fsanitize=leak',
                'msan': f'-fsanitize=memory -fPIE -pie',
                'tsan': f'-fsanitize=thread -O2 -g',
                'ubsan': f'-fsanitize=undefined'
            }[self.options.sanitizer]

            parts.append(f'-DCMAKE_C_FLAGS="{flags} -fno-omit-frame-pointer"')
            parts.append(f'-DCMAKE_CXX_FLAGS="{flags} -fno-omit-frame-pointer"')

        if self.options.colcon_build_args is not None:
            parts.append(self.options.colcon_build_args)

        return f'/bin/bash -c \'colcon build {" ".join(parts)}\''

    @property
    def volumes(self) -> Mapping[str, str]:
        return frozendict({
            **self.options.volumes,
            **Install(self.options).volumes,
            **RosdepConfig(self.options).volumes,
        })

    @memoize
    async def _main(self) -> None:
        await Container(
            self.options(
                command=self.command,
                # FIXME interactive makes us exec the docker process, so this doesn't return.
                interactive=True,
                volumes=self.volumes,
            )
        )
