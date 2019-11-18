from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from pathlib import Path

from rosdev.gen.docker.container_base import GenDockerContainerBase
from rosdev.gen.docker.image import GenDockerImage
from rosdev.gen.install_base import GenInstallBase
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenInstall(GenInstallBase):

    @staticmethod
    @memoize
    async def get_container_path(options: Options) -> Path:
        container_path = Path('/') / 'opt' / 'ros' / await GenDockerImage.get_ros_distro(options)

        log.debug(f'{GenInstall.__name__} {container_path = }')

        return container_path

    @classmethod
    async def main(cls, options: Options) -> None:
        if (
                options.docker_container_replace or
                (
                        await GenDockerContainerBase.get_id(options) !=
                        await GenDockerImage.get_id(options)
                )
        ):
            if (await cls.get_home_path(options)).exists():
                await GenDockerImage.execute(
                    command=f'sudo rm -rf {await cls.get_home_path(options)}',
                    options=options,
                )
            (await cls.get_home_path(options)).mkdir(parents=True, exist_ok=True)
