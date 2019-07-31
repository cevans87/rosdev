from dataclasses import dataclass, field, replace
from logging import getLogger
from pathlib import Path
from textwrap import dedent
from typing import Tuple, Type

from rosdev.gen.ros.overlay.setup_bash import GenRosOverlaySetupBash
from rosdev.gen.ros.underlay.setup_bash import GenRosUnderlaySetupBash
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.subprocess import exec


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerEntrypointSh(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenRosOverlaySetupBash,
        GenRosUnderlaySetupBash,
    ))

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        docker_entrypoint_sh_container_path = options.resolve_path(
            options.docker_entrypoint_sh_container_path
        )

        docker_entrypoint_sh_workspace_path = options.resolve_path(
            options.docker_entrypoint_sh_workspace_path
        )

        return replace(
            options,
            docker_entrypoint_sh_container_path=docker_entrypoint_sh_container_path,
            docker_entrypoint_sh_workspace_path=docker_entrypoint_sh_workspace_path,
        )

    @classmethod
    def get_docker_entrypoint_sh_contents(cls, options: Options) -> str:
        return dedent(fr'''
            #!/bin/bash
            set -e
            
            {
                f'source {options.ros_underlay_setup_bash_container_path}'
                if options.source_ros_underlay_setup_bash else '# Not sourcing underlay'
            }
            
            {
                f'source {options.ros_overlay_setup_bash_container_path}' if 
                options.source_ros_overlay_setup_bash else '# Not sourcing overlay'
            }
        
            exec "$@"
        ''').lstrip()

    @classmethod
    async def main(cls, options: Options) -> None:
        log.info(f'Creating docker_container_entrypoint_sh')

        await exec(f'mkdir -p {options.docker_entrypoint_sh_workspace_path.parent}')
        with open(Path(options.docker_entrypoint_sh_workspace_path), 'w') as f_out:
            f_out.write(cls.get_docker_entrypoint_sh_contents(options))
        await exec(f'chmod +x {options.docker_entrypoint_sh_workspace_path}')

        log.info(f'Created docker_container_entrypoint_sh')
