from atools import memoize
from dataclasses import dataclass
from logging import getLogger
from pathlib import Path
import shutil

from rosdev.gen.host import GenHost
from rosdev.gen.docker.image import GenDockerImage
from rosdev.gen.rosdev.home import GenRosdevHome
from rosdev.gen.rosdev.workspace import GenRosdevWorkspace
from rosdev.gen.src_base import GenSrcBase
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenSrc(Handler):

    @classmethod
    @memoize
    async def get_id(cls, options) -> str:
        id_path = await cls.get_id_path(options)
        # noinspection PyShadowingBuiltins
        id = id_path.read_text() if id_path.is_file() else ''

        log.debug(f'{cls.__name__} {id = }')

        return id

    @classmethod
    @memoize
    async def get_id_path(cls, options: Options) -> Path:
        id_path = await GenRosdevHome.get_path(options) / 'src_id'

        log.debug(f'{cls.__name__} {id_path = }')

        return id_path

    @classmethod
    @memoize
    async def get_path(cls, options: Options) -> Path:
        path = await GenRosdevHome.get_path(options) / 'src'

        log.debug(f'{cls.__name__} {path = }')

        return path

    @classmethod
    @memoize
    async def get_symlink_path(cls, options: Options) -> Path:
        symlink_path = await GenRosdevWorkspace.get_path(options) / 'src'

        log.debug(f'{cls.__name__} {symlink_path = }')

        return symlink_path

    @classmethod
    async def main(cls, options: Options) -> None:
        return
        if options.release in {'kinetic', 'melodic'}:
            log.info(f'Not installing src for ROS 1 release: {options.release = }')
            return

        if (
                (not (await GenSrcBase.get_path(options)).exists()) or
                (await GenDockerImage.get_id(options) != await cls.get_id(options))
        ):
            log.info(f'Installing src.')
            if (await GenSrcBase.get_path(options)).is_dir():
                await GenHost.execute(
                    command=f'chmod -R +w {await GenSrcBase.get_path(options)}',
                    options=options,
                )
                shutil.rmtree(await GenSrcBase.get_path(options))
            (await GenSrcBase.get_path(options)).mkdir(parents=True, exist_ok=True)
            if (await cls.get_id_path(options)).is_file():
                (await cls.get_id_path(options)).unlink()
            release_name = options.release if options.release != 'latest' else 'master'
            for command in [
                (
                        f'wget'
                        f' https://raw.githubusercontent.com/ros2/ros2/{release_name}/ros2.repos'
                        f' -O {(await GenSrcBase.get_path(options)).parent / "ros2.repos"}'
                ),
                (
                        f'vcs import --input'
                        f' {(await GenSrcBase.get_path(options)).parent / "ros2.repos"}'
                        f' {await GenSrcBase.get_path(options)}'
                ),
            ]:
                await GenDockerImage.execute(command=command, options=options)

            GenHost.write_text(
                data=await GenDockerImage.get_id(options),
                options=options,
                path=await cls.get_id_path(options),
            )
            log.info(f'Installed src.')

        if (await GenSrcBase.get_symlink_path(options)).exists():
            (await GenSrcBase.get_symlink_path(options)).unlink()
        (await GenSrcBase.get_symlink_path(options)).parent.mkdir(parents=True, exist_ok=True)
        (await GenSrcBase.get_symlink_path(options)).symlink_to(
            await GenSrcBase.get_path(options), target_is_directory=True
        )
        log.info(
            f'Linked from {await GenSrcBase.get_symlink_path(options)} to src at'
            f' {await GenSrcBase.get_path(options)}'
        )
