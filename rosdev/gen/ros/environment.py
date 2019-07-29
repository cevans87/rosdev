from atools import memoize
from dataclasses import dataclass, field
from frozendict import frozendict
from logging import getLogger
from typing import Mapping, Tuple, Type

from rosdev.gen.docker.entrypoint_sh import GenDockerEntrypointSh
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.subprocess import get_exec_lines

log = getLogger(__name__)

_ros_environment_required_keys = frozenset({
    'AMENT_PREFIX_PATH',
    'CMAKE_PREFIX_PATH',
    'COLCON_PREFIX_PATH',
    'LD_LIBRARY_PATH',
    'PATH',
    'PYTHONPATH',
    'ROS_DISTRO',
    'ROS_PYTHON_VERSION',
    'ROS_VERSION',
})


@dataclass(frozen=True)
class GenRosEnvironment(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenDockerEntrypointSh,
    ))

    @classmethod
    @memoize
    async def get_ros_environment_container(cls, options: Options) -> Mapping[str, str]:
        ros_environment_container = {}
        for line in await get_exec_lines(
                f'docker exec {options.docker_container_name} '
                f'{options.docker_entrypoint_sh_container_path} env'
        ):
            k, v = line.split('=', 1)
            if k in _ros_environment_required_keys:
                ros_environment_container[k] = v
        ros_environment_container = frozendict(ros_environment_container)

        # TODO py38 debug print
        log.debug(f'ros_environment_container: {ros_environment_container}')

        missing_keys = _ros_environment_required_keys - ros_environment_container.keys()
        assert not missing_keys, f'ros_environment missing required keys {missing_keys}'

        return ros_environment_container
