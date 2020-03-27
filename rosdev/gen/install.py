from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.docker.container_base import GenDockerContainerBase
from rosdev.gen.docker.image import GenDockerImage
from rosdev.gen.host import GenHost
from rosdev.gen.install_base import GenInstallBase
from rosdev.util.atools import memoize, memoize_db
from rosdev.util.options import Options
from rosdev.util.path import Path


log = getLogger(__name__)


@dataclass(frozen=True)
class GenInstall(GenInstallBase):

    @staticmethod
    @memoize
    async def get_container_path(options: Options) -> Path:
        container_path = Path('/opt/ros') / await GenDockerImage.get_ros_distro(options)

        log.debug(f'{GenInstall.__name__} {container_path = }')

        return container_path

    @staticmethod
    @memoize_db(keygen=lambda options: GenDockerContainerBase.get_id(options), size=1)
    async def main(options: Options) -> None:
        log.info(f'Installing install.')
        if (await GenInstallBase.get_path(options)).exists():
            await GenDockerImage.execute(
                command=f'sudo rm -rf {await GenInstallBase.get_path(options)}',
                options=options,
            )
        (await GenInstallBase.get_path(options)).mkdir(parents=True, exist_ok=True)
        
        await GenHost.execute(
            command=(
                f'docker run'
                f' --rm'
                f' --mount type=volume'
                f',dst={await GenInstall.get_container_path(options)}'
                f',volume-driver=local'
                f',volume-opt=type=none'
                f',volume-opt=o=bind'
                f',volume-opt=device={(await GenInstall.get_path(options)).resolve()}'
                f' {await GenDockerImage.get_tag(options)}'
                f' bash -c :'
            ),
            options=options,
        )
        log.info(f'Finished installing install.')
