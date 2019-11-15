from atools import memoize
from dataclasses import dataclass
from logging import getLogger
import os
from pathlib import Path
import shutil

from rosdev.gen.docker.image import GenDockerImage
from rosdev.gen.install_base import GenInstallBase
from rosdev.gen.rosdev.home import GenRosdevHome
from rosdev.util.handler import Handler
from rosdev.gen.host import GenHost
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenInstall(Handler):
    
    @classmethod
    @memoize
    async def get_id(cls, options: Options) -> str:
        id_path = await cls.get_id_path(options)
        # noinspection PyShadowingBuiltins
        id = id_path.read_text() if id_path.is_file() else ''

        log.debug(f'{cls.__name__} {id = }')

        return id

    @classmethod
    @memoize
    async def get_id_path(cls, options: Options) -> Path:
        id_path = await GenRosdevHome.get_path(options) / 'install_id'

        log.debug(f'{cls.__name__} {id_path = }')

        return id_path

    @classmethod
    async def main(cls, options: Options) -> None:
        if (
                (not (await GenInstallBase.get_path(options)).exists()) or
                (await GenDockerImage.get_id(options) != await cls.get_id(options))
        ):
            log.info(
                f'Installing install from {await GenDockerImage.get_tag(options)} docker image.'
            )
            if (await GenInstallBase.get_path(options)).is_dir():
                shutil.rmtree(await GenInstallBase.get_path(options))
            if (await cls.get_id_path(options)).is_file():
                (await cls.get_id_path(options)).unlink()
            release_name = options.release
            if release_name == 'latest':
                release_name = await GenDockerImage.execute_and_get_line(
                    command=f' ls /opt/ros 2> /dev/null',
                    options=options,
                )
            for command in [
                (
                        f'sudo mv {Path("opt", "ros") / release_name}'
                        f' {await GenInstallBase.get_path(options)}'
                ),
                (
                        f'sudo chown -R {os.getuid()}:{os.getgid()}'
                        f' {await GenInstallBase.get_path(options)}',
                ),
            ]:
                await GenDockerImage.execute(command=command, options=options)

            GenHost.write_text(
                data=await GenDockerImage.get_id(options),
                options=options,
                path=await cls.get_id_path(options),
            )
            log.info(
                f'Installed install from {await GenDockerImage.get_tag(options)} docker image.'
            )

        if (await GenInstallBase.get_symlink_path(options)).exists():
            (await GenInstallBase.get_symlink_path(options)).unlink()
        (await GenInstallBase.get_symlink_path(options)).parent.mkdir(parents=True, exist_ok=True)
        (await GenInstallBase.get_symlink_path(options)).symlink_to(
            await GenInstallBase.get_path(options), target_is_directory=True
        )
        log.info(
            f'Linked from {await GenInstallBase.get_symlink_path(options)} to install at'
            f' {await GenInstallBase.get_path(options)}'
        )
