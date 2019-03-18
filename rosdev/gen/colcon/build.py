from atools import memoize

from rosdev.gen.docker.container import Container
from rosdev.util.handler import Handler


@memoize
class Build(Handler):

    @property
    def command(self) -> str:
        parts = [f'colcon build']
        if self.options.colcon_build_args is not None:
            parts.append(self.options.colcon_build_args)

        if self.options.asan or self.options.debug:
            parts.append(f'--cmake-args -DCMAKE_BUILD_TYPE=Debug')
            if self.options.asan:
                parts.append(
                    f'-DCMAKE_CXX_FLAGS_DEBUG="-fno-omit-frame-pointer -fsanitize=address"')

        return ' '.join(parts)

    @memoize
    async def exit_code(self) -> int:
        parts = [f'colcon build']
        if self.options.colcon_build_args is not None:
            parts.append(self.options.colcon_build_args)

        if self.options.asan or self.options.debug:
            parts.append(f'--cmake-args -DCMAKE_BUILD_TYPE=Debug')
            if self.options.asan:
                parts.append(
                    f'-DCMAKE_CXX_FLAGS_DEBUG="-fno-omit-frame-pointer -fsanitize=address"')

        command = ' '.join(parts)
        return await Container(self.settings(command=command)).exit_code()

    @memoize
    async def must_succeed(self) -> None:
        if await self.exit_code() != 0:
            raise Exception('Build failed.')

    @memoize
    async def _main(self) -> None:
        await self.exit_code()
