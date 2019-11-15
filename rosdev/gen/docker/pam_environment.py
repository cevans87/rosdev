from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from pathlib import Path

from rosdev.gen.docker.container import GenDockerContainer
from rosdev.gen.home import GenHome
from rosdev.gen.host import GenHost
from rosdev.gen.rosdev.workspace import GenRosdevWorkspace
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerPamEnvironment(Handler):

    @classmethod
    @memoize
    async def get_symlink_container_path(cls, options: Options) -> Path:
        symlink_container_path = await GenHome.get_path(options) / '.pam_environment'
        
        log.debug(f'{__package__} {symlink_container_path}')
        
        return symlink_container_path

    @classmethod
    @memoize
    async def get_path(cls, options: Options) -> Path:
        path = await GenRosdevWorkspace.get_path(options) / 'docker_pam_environment'
        
        log.debug(f'{__package__} {path = }')
        
        return path

    @classmethod
    async def main(cls, options: Options) -> None:
        log.info(f'Creating pam_environment')

        docker_ssh_pam_environment_lines = []
        for k, v in (await GenDockerContainer.get_environment(options)).items():
            i = 0
            while len(v) > 512:
                docker_ssh_pam_environment_lines.append(f'{k}_{i} DEFAULT="{v[:512]}"')
                i += 1
                v = v[512:]

            if i > 0 and v:
                docker_ssh_pam_environment_lines.append(f'{k}_{i} DEFAULT="{v}"')
                i += 1

            if i == 0:
                docker_ssh_pam_environment_lines.append(f'{k} DEFAULT="{v}"')
            else:
                docker_ssh_pam_environment_lines.append(
                    f'{k} DEFAULT="{"".join([f"${{{k}_{j}}}" for j in range(i)])}"'
                )

            # FIXME LD_PRELOAD should happen at runtime, not compile time
            #if self.options.sanitizer is not None:
            #    if self.options.sanitizer == 'asan':
            #        docker_ssh_pam_environment_lines.append(
            #            f'LD_PRELOAD DEFAULT='
            #            f'/usr/lib/{get_machine(self.options.architecture)}-linux-gnu/'
            #            f'libasan.so.4'
            #        )
            #    else:
            #        docker_ssh_pam_environment_lines.append(
            #            f'LD_PRELOAD DEFAULT='
            #            f'/usr/lib/{get_machine(self.options.architecture)}-linux-gnu/'
            #            f'lib{self.options.sanitizer}.so.0'
            #        )

        GenHost.write_text(
            data='\n'.join(docker_ssh_pam_environment_lines),
            options=options,
            path=await cls.get_path(options),
        )
        await GenDockerContainer.execute(
            command=(
                f'ln -f -s '
                f'{await cls.get_path(options)} '
                f'{await cls.get_symlink_container_path(options)}'
            ),
            err_ok=True,
            options=options,
        )

        log.info(f'Created pam_environment')
