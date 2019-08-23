from dataclasses import dataclass
from logging import getLogger
from pathlib import Path
from textwrap import dedent

from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.subprocess import execute_command


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerEntrypointSh(Handler):

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        # TODO py38 debug print
        log.debug(
            f'docker_entrypoint_sh_container_path: '
            f'{options.docker_entrypoint_sh_container_path}'
        )
        log.debug(
            f'docker_entrypoint_sh_workspace_path: '
            f'{options.docker_entrypoint_sh_workspace_path}'
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

        options.write_text(
            path=options.docker_entrypoint_sh_workspace_path,
            text=cls.get_docker_entrypoint_sh_contents(options)
        )
        # Make executable
        options.docker_entrypoint_sh_workspace_path.chmod(0o755)

        log.info(f'Created docker_container_entrypoint_sh')
