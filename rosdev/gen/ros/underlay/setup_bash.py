from dataclasses import dataclass, field, replace
from frozendict import frozendict
from logging import getLogger
from typing import Tuple, Type

from rosdev.gen.ros.install import GenRosInstall
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenRosUnderlaySetupBash(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenRosInstall,
    ))

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        ros_underlay_setup_bash_container_path = options.resolve_path(
            options.ros_underlay_setup_bash_container_path
        )
        ros_underlay_setup_bash_workspace_path = options.resolve_path(
            options.ros_underlay_setup_bash_workspace_path
        )

        docker_container_volumes = dict(options.docker_container_volumes)
        docker_container_volumes[ros_underlay_setup_bash_workspace_path] = (
            ros_underlay_setup_bash_container_path
        )
        docker_container_volumes = frozendict(docker_container_volumes)

        return replace(
            options,
            docker_container_volumes=docker_container_volumes,
            ros_underlay_setup_bash_container_path=ros_underlay_setup_bash_container_path,
            ros_underlay_setup_bash_workspace_path=ros_underlay_setup_bash_workspace_path,
        )
