from dataclasses import dataclass, field, replace
from frozendict import frozendict
from logging import getLogger
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import Tuple, Type

from rosdev.gen.base import GenBase
from rosdev.gen.docker.image import GenDockerImage
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenSrc(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenBase,
        GenDockerImage,
    ))

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        docker_container_volumes = dict(options.docker_container_volumes)
        docker_container_volumes[options.src_path] = options.src_path
        docker_container_volumes[options.src_symlink_path] = options.src_symlink_path
        docker_container_volumes = frozendict(docker_container_volumes)

        return replace(
            options,
            docker_container_volumes=docker_container_volumes,
        )

    @classmethod
    async def main(cls, options: Options) -> None:
        # FIXME this only detects that src exists, but not the newest src. Associate the src with
        #  the id of the image with which it was installed.
        if not options.src_path.exists():
            log.info(f'Installing src.')
            release_name = options.release
            if release_name == 'latest':
                release_name = await cls.execute_shell_host_and_get_line(
                    command=(
                        f'docker run --rm {options.docker_image_base_tag} ls /opt/ros 2> /dev/null'
                    ),
                )
            with TemporaryDirectory() as temp_dir:
                staging_path = Path(temp_dir, 'src')
                staging_path.mkdir(parents=True, exist_ok=True)
                options.src_path.parent.mkdir(parents=True, exist_ok=True)
                for command in [
                    (
                            f'wget https://raw.githubusercontent.com/ros2/ros2/'
                            f'{release_name}/ros2.repos '
                            f'-O {staging_path.parent / "ros2.repos"}'
                    ),
                    f'vcs import --input {staging_path.parent / "ros2.repos"} {staging_path}',
                    f'sudo chmod -R -w {staging_path}',
                    f'sudo mv {staging_path} {options.src_path}'
                ]:
                    await cls.execute_host(
                        command=(
                            f'docker run --rm '
                            f'{options.docker_environment_flags} '
                            f'-v {staging_path.parent}:{staging_path.parent} '
                            f'-v {options.src_path.parent}:{options.src_path.parent} '
                            f'{options.docker_image_tag} '
                            f'{command}'
                        )
                    )
            log.info(f'Installed src.')

        options.src_symlink_path.parent.mkdir(parents=True, exist_ok=True)
        options.src_symlink_path.symlink_to(options.src_path, target_is_directory=True)
        log.info(f'Linked src {options.src_symlink_path} -> {options.src_path}')
