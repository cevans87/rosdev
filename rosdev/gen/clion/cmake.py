from atools import memoize
from dataclasses import dataclass
from frozendict import frozendict
from logging import getLogger
from pathlib import Path

from rosdev.gen.rosdev.config import Config as RosdevConfig
from rosdev.gen.install import Install
from rosdev.util.handler import Handler
from rosdev.util.lookup import get_machine
from rosdev.util.subprocess import exec, get_shell_lines


log = getLogger(__name__)


# FIXME move this to rosdev/gen/pam_environment.py


@memoize
@dataclass(frozen=True)
class Cmake(Handler):

    @property
    def container_pam_environment_path(self) -> str:
        return f'{Path.home()}/.pam_environment'

    @property
    def local_pam_environment_path_base(self) -> str:
        return RosdevConfig(self.options).local_path

    @property
    def local_pam_environment_path(self) -> str:
        return f'{self.local_pam_environment_path_base}/pam_environment'

    @property
    def volumes(self) -> frozendict:
        return frozendict({
            **self.options.volumes,
            self.local_pam_environment_path: self.container_pam_environment_path
        })

    @memoize
    async def get_environ(self) -> frozendict:
        await Install(self.options)

        command = f'bash -c \''
        if self.options.global_setup is not None:
            command += f'source {Path(self.options.global_setup).expanduser().absolute()} && '
        if self.options.local_setup is not None:
            command += f'source {Path(self.options.local_setup).expanduser().absolute()} && '
        command += 'env\''

        lines = await get_shell_lines(command)
        environ = {}
        for line in lines:
            if line:
                k, v = line.split('=', 1)
                if k in {
                    'AMENT_PREFIX_PATH',
                    'CMAKE_PREFIX_PATH',
                    'COLCON_PREFIX_PATH',
                    'LD_LIBRARY_PATH',
                    'PATH',
                    'PYTHONPATH',
                    'ROS_DISTRO',
                    'ROS_PYTHON_VERSION',
                    'ROS_VERSION',
                }:
                    log.debug(f'Writing env to .pam_environment "{k}"')
                    environ[k] = v
                else:
                    log.debug(f'Omitting env in .pam_environment "{k}"')

        return frozendict(environ)

    @memoize
    async def _main(self) -> None:
        await Install(self.options)

        pam_environment_lines = []
        for k, v in (await self.get_environ()).items():
            i = 0
            while len(v) > 512:
                pam_environment_lines.append(f'{k}_{i} DEFAULT="{v[:512]}"')
                i += 1
                v = v[512:]

            if i > 0 and v:
                pam_environment_lines.append(f'{k}_{i} DEFAULT="{v}"')
                i += 1

            if i == 0:
                pam_environment_lines.append(f'{k} DEFAULT="{v}"')
            else:
                pam_environment_lines.append(
                    f'{k} DEFAULT="{"".join([f"${{{k}_{j}}}" for j in range(i)])}"'
                )

        # FIXME LD_PRELOAD should happen at runtime, not compile time
        #if self.options.sanitizer is not None:
        #    if self.options.sanitizer == 'asan':
        #        pam_environment_lines.append(
        #            f'LD_PRELOAD DEFAULT='
        #            f'/usr/lib/{get_machine(self.options.architecture)}-linux-gnu/'
        #            f'libasan.so.4'
        #        )
        #    else:
        #        pam_environment_lines.append(
        #            f'LD_PRELOAD DEFAULT='
        #            f'/usr/lib/{get_machine(self.options.architecture)}-linux-gnu/'
        #            f'lib{self.options.sanitizer}.so.0'
        #        )

        await exec(f'mkdir -p {self.local_pam_environment_path_base}')
        with open(self.local_pam_environment_path, 'w') as pam_environment_f_out:
            pam_environment_f_out.write('\n'.join(pam_environment_lines))

        log.info(f'PAM environment written to {self.local_pam_environment_path}')
