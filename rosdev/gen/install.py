from atools import memoize
from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.docker.container_base import GenDockerContainerBase
from rosdev.gen.docker.image import GenDockerImage
from rosdev.gen.host import GenHost
from rosdev.gen.install_base import GenInstallBase
from rosdev.util.options import Options
from rosdev.util.path import Path


log = getLogger(__name__)


@dataclass(frozen=True)
class GenInstall(GenInstallBase):

    @staticmethod
    @memoize
    async def get_container_path(options: Options) -> Path:
        container_path = Path('/') / 'opt' / 'ros' / await GenDockerImage.get_ros_distro(options)

        log.debug(f'{GenInstall.__name__} {container_path = }')

        return container_path

    @staticmethod
    async def main(options: Options) -> None:
        if (
                options.docker_container_replace or
                (not (await GenInstallBase.get_home_path(options)).exists()) or
                (not (await GenInstallBase.get_id(options))) or
                (
                        await GenInstallBase.get_id(options) !=
                        await GenDockerContainerBase.get_id(options)
                )
        ):
            log.info(f'Installing install.')
            if (await GenInstallBase.get_id_path(options)).exists():
                (await GenInstallBase.get_id_path(options)).unlink()
            if (await GenInstallBase.get_home_path(options)).exists():
                await GenDockerImage.execute(
                    command=f'sudo rm -rf {await GenInstallBase.get_home_path(options)}',
                    options=options,
                )
            (await GenInstallBase.get_home_path(options)).mkdir(parents=True, exist_ok=True)

            GenHost.write_text(
                data=await GenDockerImage.get_id(options),
                options=options,
                path=await GenInstallBase.get_id_path(options),
            )
