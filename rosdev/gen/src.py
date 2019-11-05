from atools import memoize
from dataclasses import dataclass, field, replace
from frozendict import frozendict
from logging import getLogger
import shutil
from typing import Tuple, Type

from rosdev.gen.base import GenBase
from rosdev.gen.host import GenHost
from rosdev.gen.docker.image import GenDockerImage
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenSrc(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenBase,
        GenDockerImage,
        GenHost,
    ))

    @classmethod
    @memoize
    async def get_id(cls, options) -> str:
        return options.src_id_path.read_text() if options.src_id_path.is_file() else ''

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        docker_container_volumes = dict(options.docker_container_volumes)
        docker_container_volumes[options.src_id_path] = options.src_id_path
        docker_container_volumes[options.src_path] = options.src_path
        docker_container_volumes = frozendict(docker_container_volumes)

        return replace(
            options,
            docker_container_volumes=docker_container_volumes,
        )

    @classmethod
    async def main(cls, options: Options) -> None:
        if options.release in {'kinetic', 'melodic'}:
            log.info(f'Not installing src for ROS 1 release: {options.release = }')
            return

        if (
                (not options.src_path.exists()) or
                (await GenDockerImage.get_id(options) != await cls.get_id(options))
        ):
            log.info(f'Installing src.')
            if options.src_path.is_dir():
                await GenHost.execute(command=f'chmod -R +w {options.src_path}', options=options)
                shutil.rmtree(options.src_path)
            options.src_path.mkdir(parents=True, exist_ok=True)
            if options.src_id_path.is_file():
                options.src_id_path.unlink()
            release_name = options.release if options.release != 'latest' else 'master'
            for command in [
                (
                        f'wget https://raw.githubusercontent.com/ros2/ros2/'
                        f'{release_name}/ros2.repos '
                        f'-O {options.home_rosdev_path.parent / "ros2.repos"}'
                ),
                (
                        f'vcs import --input {options.home_rosdev_path.parent / "ros2.repos"}'
                        f' {options.src_path}'
                ),
            ]:
                await GenDockerImage.execute(command=command, options=options)

            GenHost.write_text(
                data=await GenDockerImage.get_id(options),
                options=options,
                path=options.src_id_path,
            )
            log.info(f'Installed src.')

        if options.src_symlink_path.exists():
            options.src_symlink_path.unlink()
        options.src_symlink_path.parent.mkdir(parents=True, exist_ok=True)
        options.src_symlink_path.symlink_to(options.src_path, target_is_directory=True)
        log.info(f'Linked from {options.src_symlink_path} to src at {options.src_path}')
