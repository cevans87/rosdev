from atools import memoize
from dataclasses import dataclass
from frozendict import frozendict
from logging import getLogger
from pathlib import Path

from rosdev.gen.rosdev.config import Config as RosdevConfig
from rosdev.gen.install import Install
from rosdev.util.handler import Handler
from rosdev.util.subprocess import exec, get_shell_lines


log = getLogger(__name__)


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

        if self.options.clean and self.options.local_setup_path is None:
            command = 'bash -c \'env\''
        else:
            command = f'bash -c \''
            if not self.options.clean:
                command += f'source {Install(self.options).container_path}/setup.bash && '
            if self.options.local_setup_path is not None:
                command += f'source {self.options.local_setup_path} && '
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
                    environ[k] = v

        return frozendict(environ)

    @memoize
    async def _main(self) -> None:
        await Install(self.options)

        pam_environment_lines = []
        for k, v in (await self.get_environ()).items():
            i = 0
            # TODO unset these temp environment variables in rosdev_entrypoint.sh
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

        await exec(f'mkdir -p {self.local_pam_environment_path_base}')
        with open(self.local_pam_environment_path, 'w') as pam_environment_f_out:
            pam_environment_f_out.write('\n'.join(pam_environment_lines))

        log.info(f'PAM environment written to {self.local_pam_environment_path}')
