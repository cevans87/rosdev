from atools import memoize
from dataclasses import dataclass, field
from frozendict import frozendict
from logging import getLogger
from typing import Dict, Mapping, Tuple, Type

from rosdev.gen.docker.core import GenDockerCore
from rosdev.util.handler import Handler
from rosdev.util.options import Options

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
class GenEnvironment(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenDockerCore,
    ))

    @classmethod
    @memoize
    async def get_environment_container(cls, options: Options) -> Mapping[str, str]:
        environment: Dict[str, str] = {}
        for line in await cls.exec_container(options=options, command='env'):
            try:
                k, v = line.split('=', 1)
            except ValueError:
                continue
            else:
                if k in _ros_environment_required_keys:
                    environment[k] = v

        # TODO py38 debug print
        log.debug(f'container_environment: {environment}')

        missing_keys = _ros_environment_required_keys - environment.keys()
        if missing_keys:
            log.warning(f'ros_environment missing keys {missing_keys}')

        return frozendict(environment)
