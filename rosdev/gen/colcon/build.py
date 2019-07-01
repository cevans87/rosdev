from atools import memoize
from frozendict import frozendict
from typing import Mapping

from rosdev.gen.docker.container import Container
from rosdev.gen.install import Install
from rosdev.gen.rosdep.config import Config as RosdepConfig
from rosdev.gen.rosdep.install import Install as RosdepInstall
from rosdev.util.handler import Handler
from rosdev.util.options import Options


@memoize
class Build(Handler):

    @property
    def command(self) -> str:
        parts = [f'--cmake-args -DCMAKE_BUILD_TYPE={self.options.build_type}']

        # TODO use sanitizer mixins created specifically for this. Requires that mixins be built
        #  into docker image.
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
    @memoize
    def options(self) -> Options:
        return super().options(
            command=self.command,
            container_name=RosdepInstall(super().options).container_name,
            replace_named_container=False,
            volumes=self.volumes,
        )

    @property
    def volumes(self) -> Mapping[str, str]:
        return frozendict({
            **self.options.volumes,
            **Install(self.options).volumes,
            **RosdepConfig(self.options).volumes,
        })

    @memoize
    async def _main(self) -> None:
        await RosdepInstall(self.options)
        await Container(self.options)
