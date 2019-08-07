from dataclasses import dataclass, field
from logging import getLogger
import os
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import Tuple, Type

from rosdev.gen.docker.base import GenDockerBase
from rosdev.gen.docker.image import GenDockerImage
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerInstall(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenDockerBase,
        GenDockerImage,
    ))

    @classmethod
    async def main(cls, options: Options) -> None:
        if options.release not in {'kinetic', 'melodic'}:
            log.info(f'Bypassing ROS 1 install container copy')
            return

        if options.install_universal_path.exists():
            log.info(f'Found install cached at {options.install_universal_path}')
            return

        log.info(f'Installing from {options.docker_image_base_tag} docker image')
        with TemporaryDirectory() as temp_dir:
            staging_path = Path(temp_dir, 'install')
            options.install_universal_path.parent.mkdir(parents=True, exist_ok=True)
            for cmd in [
                f'mv /opt/ros/{options.release} {staging_path}',
                f'chown -R {os.getuid()}:{os.getgid()} {staging_path}',
                f'chmod -R -w {staging_path}',
            ]:
                await cls.exec_workspace(
                    options=options,
                    cmd=(
                        f'docker run --rm -v '
                        f'{options.install_universal_path.parent}:'
                        f'{options.install_universal_path.parent} '
                        f'{options.docker_image_base_tag} '
                        f'{cmd}'
                    )
                )
            staging_path.rename(options.install_universal_path)

        if options.install_workspace_path.exists():
            options.install_workspace_path.unlink()
        options.install_workspace_path.parent.mkdir(parents=True, exist_ok=True)
        options.install_workspace_path.symlink_to(
            options.install_universal_path,
            target_is_directory=True,
        )
        log.info(f'Installed from {options.docker_image_base_tag} docker image')
