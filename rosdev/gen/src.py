from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.docker.container_base import GenDockerContainerBase
from rosdev.gen.docker.image import GenDockerImage
from rosdev.gen.host import GenHost
from rosdev.gen.src_base import GenSrcBase
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenSrc(GenSrcBase):

    @staticmethod
    async def main(options: Options) -> None:
        if options.release in {'kinetic', 'melodic'}:
            log.info(f'Not installing src for ROS 1 release "{options.release}".')
            return
        elif (
                options.docker_container_replace or
                (not (await GenSrcBase.get_home_path(options)).exists()) or
                (not (await GenSrcBase.get_id(options))) or
                (
                        await GenSrcBase.get_id(options) !=
                        await GenDockerContainerBase.get_id(options)
                )
        ):
            log.info(f'Installing src.')
            if (await GenSrcBase.get_id_path(options)).exists():
                (await GenSrcBase.get_id_path(options)).unlink()
            if (await GenSrcBase.get_home_path(options)).exists():
                await GenDockerImage.execute(
                    command=f'sudo rm -rf {await GenSrcBase.get_home_path(options)}',
                    options=options,
                )

            release_name = options.release if options.release != 'latest' else 'master'
            (await GenSrcBase.get_home_path(options)).mkdir(parents=True)
            for command in [
                (
                        f'wget'
                        f' https://raw.githubusercontent.com/ros2/ros2/{release_name}/ros2.repos'
                        f' -O {(await GenSrcBase.get_home_path(options)).parent / "ros2.repos"}'
                ),
                (
                        f'vcs import --input'
                        f' {(await GenSrcBase.get_home_path(options)).parent / "ros2.repos"}'
                        f' {await GenSrcBase.get_home_path(options)}'
                ),
            ]:
                await GenDockerImage.execute(command=command, options=options)

            GenHost.write_text(
                data=await GenDockerImage.get_id(options),
                options=options,
                path=await GenSrcBase.get_id_path(options),
            )
