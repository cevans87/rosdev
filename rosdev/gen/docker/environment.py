from atools import memoize
from dataclasses import dataclass, field
from frozendict import frozendict
from logging import getLogger
from typing import Mapping, Tuple, Type

from rosdev.gen.docker.base import GenDockerBase
from rosdev.gen.docker.container import GenDockerContainer
from rosdev.gen.docker.install import GenDockerInstall
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
class GenDockerEnvironment(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenDockerBase,
        GenDockerContainer,
        GenDockerInstall,
    ))

    @classmethod
    @memoize
    async def get_ros_environment_container(cls, options: Options) -> Mapping[str, str]:
        ros_container_environment = {}
        for line in await get_exec_lines(
                f'docker exec {options.docker_container_name} '
                f'{options.docker_entrypoint_sh_container_path} env'
        ):
            try:
                k, v = line.split('=', 1)
            except ValueError:
                continue
            else:
                if k in _ros_environment_required_keys:
                    ros_container_environment[k] = v
        ros_container_environment = frozendict(ros_container_environment)

        # TODO py38 debug print
        log.debug(f'ros_container_environment: {ros_container_environment}')

        missing_keys = _ros_environment_required_keys - ros_container_environment.keys()
        if missing_keys:
            log.warning(f'ros_environment missing keys {missing_keys}')

        return ros_container_environment
