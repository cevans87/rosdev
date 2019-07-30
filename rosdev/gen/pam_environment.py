from dataclasses import dataclass, field, replace
from logging import getLogger
from typing import Tuple, Type

from rosdev.gen.home import GenHome
from rosdev.gen.ros.environment import GenRosEnvironment
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.subprocess import exec


log = getLogger(__name__)


@dataclass(frozen=True)
class GenPamEnvironment(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenHome,
        GenRosEnvironment,
    ))

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        pam_environment_container_path = options.resolve_path(
            options.pam_environment_container_path
        )

        pam_environment_workspace_path = options.resolve_path(
            options.pam_environment_workspace_path
        )

        return replace(
            options,
            pam_environment_container_path=pam_environment_container_path,
            pam_environment_workspace_path=pam_environment_workspace_path,
        )

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        # TODO py38 debug print
        log.debug(f'pam_environment_container_path: {options.pam_environment_container_path}')
        log.debug(f'pam_environment_workspace_path: {options.pam_environment_workspace_path}')

    @classmethod
    async def main(cls, options: Options) -> None:
        log.info(f'Creating pam_environment')

        if not options.pam_environment_workspace_path.is_file():
            pam_environment_lines = []
            for k, v in (await GenRosEnvironment.get_ros_environment_container(options)).items():
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

            await exec(f'mkdir -p {options.pam_environment_workspace_path.parent}', err_ok=True)
            with open(str(options.pam_environment_workspace_path), 'w') as f_out:
                f_out.write('\n'.join(pam_environment_lines))
            await exec(
                f'docker exec {options.docker_container_name} '
                f'ln -s {options.pam_environment_container_path} '
                f'{options.home_container_path}/.pam_environment'
            )

        log.info(f'Created pam_environment')
