from dataclasses import dataclass, field, replace
from frozendict import frozendict
from logging import getLogger
from typing import Tuple, Type

from rosdev.gen.docker.container import GenDockerContainer
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
        GenDockerContainer,
        GenDockerEntrypointSh,
    ))

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        ros_container_environment = dict(options.ros_container_environment)
        for line in await get_exec_lines(
                f'docker exec {options.docker_container_name} '
                f'{options.docker_entrypoint_sh_container_path} env'
        ):
            k, v = line.split('=', 1)
            if k in _ros_environment_required_keys:
                ros_container_environment[k] = v
        ros_container_environment = frozendict(ros_container_environment)

        return replace(options, ros_container_environment=ros_container_environment)

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        # TODO py38 debug print
        log.debug(f'ros_container_environment: {options.ros_container_environment}')

        missing_keys = _ros_environment_required_keys - options.ros_container_environment.keys()
        assert not missing_keys, f'ros_environment missing required keys {missing_keys}'
