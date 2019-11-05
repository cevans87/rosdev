from atools import memoize
from dataclasses import dataclass, field, replace
from frozendict import frozendict
from logging import getLogger
import os
from pathlib import Path
import shutil
from typing import Tuple, Type

from rosdev.gen.base import GenBase
from rosdev.gen.docker.image import GenDockerImage
from rosdev.util.handler import Handler
from rosdev.gen.host import GenHost
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenInstall(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenBase,
        GenDockerImage,
    ))

    @classmethod
    @memoize
    async def get_id(cls, options) -> str:
        return options.install_id_path.read_text() if options.install_id_path.is_file() else ''

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        docker_container_volumes = dict(options.docker_container_volumes)
        docker_container_volumes[options.install_path] = options.install_path
        docker_container_volumes[options.install_symlink_path] = options.install_symlink_path
        docker_container_volumes = frozendict(docker_container_volumes)

        return replace(
            options,
            docker_container_volumes=docker_container_volumes,
        )

    @classmethod
    async def main(cls, options: Options) -> None:
        if (
                (not options.install_path.exists()) or
                (await GenDockerImage.get_id(options) != await cls.get_id(options))
        ):
            log.info(f'Installing install from {options.docker_image_base_tag} docker image')
            if options.install_path.is_dir():
                shutil.rmtree(options.install_path)
            if options.install_id_path.is_file():
                options.install_id_path.unlink()
            release_name = options.release
            if release_name == 'latest':
                # FIXME move to rosdev.gen.release
                release_name = await GenHost.execute_shell_and_get_line(
                    command=(
                        f'docker run --rm {options.docker_image_base_tag} ls /opt/ros 2> /dev/null'
                    ),
                    options=options,
                )
            for command in [
                f'sudo mv {Path("opt", "ros") / release_name} {options.install_path}',
                f'sudo chown -R {os.getuid()}:{os.getgid()} {options.install_path}',
            ]:
                await GenDockerImage.execute(command=command, options=options)

            GenHost.write_text(
                data=await GenDockerImage.get_id(options),
                options=options,
                path=options.install_id_path,
            )
            log.info(f'Installed install from {options.docker_image_base_tag} docker image')

        if options.install_symlink_path.exists():
            options.install_symlink_path.unlink()
        options.install_symlink_path.parent.mkdir(parents=True, exist_ok=True)
        options.install_symlink_path.symlink_to(options.install_path, target_is_directory=True)
        log.info(f'Linked from {options.install_symlink_path} to install at {options.install_path}')
