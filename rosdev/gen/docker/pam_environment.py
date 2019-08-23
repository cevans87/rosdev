from dataclasses import dataclass, field, replace
from frozendict import frozendict
from logging import getLogger
from typing import Tuple, Type

from rosdev.gen.docker.base import GenDockerBase
from rosdev.gen.docker.environment import GenDockerEnvironment
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerPamEnvironment(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenDockerBase,
        GenDockerEnvironment,
    ))

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        docker_container_volumes = dict(options.docker_container_volumes)
        docker_container_volumes[options.docker_pam_environment_workspace_path] = (
            options.docker_pam_environment_container_path
        )
        docker_container_volumes = frozendict(options.docker_container_volumes)

        return replace(options, docker_container_volumes=docker_container_volumes)

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        # TODO py38 debug print
        log.debug(
            f'docker_pam_environment_container_path: '
            f'{options.docker_pam_environment_container_path}'
        )
        log.debug(
            f'docker_pam_environment_workspace_path: '
            f'{options.docker_pam_environment_workspace_path}'
        )

    @classmethod
    async def main(cls, options: Options) -> None:
        log.info(f'Creating pam_environment')

        docker_pam_environment_lines = []
        for k, v in (await GenDockerEnvironment.get_ros_environment_container(options)).items():
            i = 0
            while len(v) > 512:
                docker_pam_environment_lines.append(f'{k}_{i} DEFAULT="{v[:512]}"')
                i += 1
                v = v[512:]

            if i > 0 and v:
                docker_pam_environment_lines.append(f'{k}_{i} DEFAULT="{v}"')
                i += 1

            if i == 0:
                docker_pam_environment_lines.append(f'{k} DEFAULT="{v}"')
            else:
                docker_pam_environment_lines.append(
                    f'{k} DEFAULT="{"".join([f"${{{k}_{j}}}" for j in range(i)])}"'
                )

            # FIXME LD_PRELOAD should happen at runtime, not compile time
            #if self.options.sanitizer is not None:
            #    if self.options.sanitizer == 'asan':
            #        docker_pam_environment_lines.append(
            #            f'LD_PRELOAD DEFAULT='
            #            f'/usr/lib/{get_machine(self.options.architecture)}-linux-gnu/'
            #            f'libasan.so.4'
            #        )
            #    else:
            #        docker_pam_environment_lines.append(
            #            f'LD_PRELOAD DEFAULT='
            #            f'/usr/lib/{get_machine(self.options.architecture)}-linux-gnu/'
            #            f'lib{self.options.sanitizer}.so.0'
            #        )

            options.write_text(
                path=options.docker_pam_environment_workspace_path,
                text='\n'.join(docker_pam_environment_lines)
            )
            await cls.exec_container(
                options=options,
                cmd=(
                    f'ln -f -s {options.docker_pam_environment_container_path} '
                    f'{options.home_container_path}/.pam_environment'
                )
            )

        log.info(f'Created pam_environment')