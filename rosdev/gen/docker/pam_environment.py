from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.docker.container import GenDockerContainer
from rosdev.gen.store import GenStore
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.path import Path


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerPamEnvironment(Handler):

    @staticmethod
    @memoize
    async def get_symlink_container_path(options: Options) -> Path:
        symlink_container_path = Path.home() / '.pam_environment'
        
        log.debug(f'{__class__.__name__} {symlink_container_path}')
        
        return symlink_container_path

    @staticmethod
    @memoize
    async def get_path(options: Options) -> Path:
        path = await GenStore.get_path(options) / 'docker' / 'rosdev_docker_pam_environment'

        log.debug(f'{__class__.__name__} {path = }')
        
        return path

    @staticmethod
    @memoize
    async def main(options: Options) -> None:
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

        (await GenDockerPamEnvironment.get_path(options)).write_text(
            data='\n'.join(docker_ssh_pam_environment_lines),
        )
        await GenDockerContainer.execute(
            command=f'rm -f {await GenDockerPamEnvironment.get_symlink_container_path(options)}',
            options=options,
        )
        await GenDockerContainer.execute(
            command=(
                f' ln -s'
                f' {await GenDockerPamEnvironment.get_path(options)}'
                f' {await GenDockerPamEnvironment.get_symlink_container_path(options)}'
            ),
            err_ok=True,
            options=options,
        )

        log.info(f'Created pam_environment')
