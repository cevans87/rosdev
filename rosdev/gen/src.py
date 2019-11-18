from dataclasses import dataclass
from logging import getLogger

from rosdev.gen.docker.image import GenDockerImage
from rosdev.gen.src_base import GenSrcBase
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenSrc(GenSrcBase):

    @classmethod
    async def main(cls, options: Options) -> None:
        if options.release in {'kinetic', 'melodic'}:
            log.info(f'Not installing src for ROS 1 release "{options.release}".')
            return
            # TODO add src-pull flag
        elif not (await GenSrc.get_home_path(options)).exists():
            log.info('Src does not exist.')
        elif not options.src_replace:
            log.debug('Src already exists.')
            return

        log.info(f'Installing src.')
        if (await GenSrc.get_home_path(options)).exists():
            await GenDockerImage.execute(
                command=f'sudo rm -rf {await cls.get_home_path(options)}',
                options=options,
            )

        # FIXME if this crashes partway through, we'll never fix the missing sources on
        #  subsequent runs. Add back in the src_id logic.
        (await GenSrc.get_home_path(options)).mkdir(parents=True)

        release_name = options.release if options.release != 'latest' else 'master'
        for command in [
            (
                    f'wget'
                    f' https://raw.githubusercontent.com/ros2/ros2/{release_name}/ros2.repos'
                    f' -O {(await GenSrc.get_home_path(options)).parent / "ros2.repos"}'
            ),
            (
                    f'vcs import --input'
                    f' {(await GenSrc.get_home_path(options)).parent / "ros2.repos"}'
                    f' {await GenSrc.get_home_path(options)}'
            ),
        ]:
            await GenDockerImage.execute(command=command, options=options)
