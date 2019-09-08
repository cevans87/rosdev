from dataclasses import dataclass, replace
from frozendict import frozendict
from logging import getLogger
from textwrap import dedent

from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerEntrypointSh(Handler):

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        docker_container_volumes = dict(options.docker_container_volumes)
        docker_container_volumes[options.docker_entrypoint_sh_overlay_sh_workspace_path] = (
            options.docker_entrypoint_sh_overlay_sh_container_path
        )
        docker_container_volumes[options.docker_entrypoint_sh_underlay_sh_workspace_path] = (
            options.docker_entrypoint_sh_underlay_sh_container_path
        )
        docker_container_volumes[options.docker_entrypoint_sh_workspace_path] = (
            options.docker_entrypoint_sh_container_path
        )
        docker_container_volumes[options.docker_entrypoint_sh_workspace_path] = (
            options.docker_entrypoint_sh_container_path
        )
        docker_container_volumes = frozendict(docker_container_volumes)

        return replace(options, docker_container_volumes=docker_container_volumes)

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        # TODO py38 debug print
        log.debug(
            f'docker_entrypoint_sh_quick_overlay: {options.docker_entrypoint_sh_quick_overlay}'
        )
        log.debug(
            f'docker_entrypoint_sh_quick_underlay: {options.docker_entrypoint_sh_quick_underlay}'
        )

        log.debug(
            f'docker_entrypoint_sh_setup_overlay: {options.docker_entrypoint_sh_setup_overlay}'
        )
        log.debug(
            f'docker_entrypoint_sh_setup_underlay: {options.docker_entrypoint_sh_setup_underlay}'
        )
        log.debug(
            f'docker_entrypoint_sh_setup_underlay_container_path: '
            f'{options.docker_entrypoint_sh_setup_underlay_container_path}'
        )
        log.debug(
            f'docker_entrypoint_sh_setup_underlay_workspace_path: '
            f'{options.docker_entrypoint_sh_setup_underlay_workspace_path}'
        )
        log.debug(
            f'docker_entrypoint_sh_container_path: {options.docker_entrypoint_sh_container_path}'
        )
        log.debug(
            f'docker_entrypoint_sh_workspace_path: {options.docker_entrypoint_sh_workspace_path}'
        )

    @classmethod
    def get_docker_entrypoint_sh_contents(cls, options: Options) -> str:
        return dedent(fr'''
            #!/bin/bash
            set -e

            docker_entrypoint_sh_quick_underlay="{options.docker_entrypoint_sh_quick_underlay}"
            docker_entrypoint_sh_setup_underlay="{options.docker_entrypoint_sh_setup_underlay}"
            
            if [[ $docker_entrypoint_sh_setup_underlay = "False" ]]
            then
                echo "Not sourcing underlay from ROS install."
            elif
                [[ $docker_entrypoint_sh_quick_underlay = "True" ]] && \
                [[ -f "{options.docker_entrypoint_sh_underlay_sh_container_path}" ]] && \
                [[ {options.docker_entrypoint_sh_underlay_sh_container_path} -nt \
                   {options.docker_entrypoint_sh_setup_underlay_container_path} ]]
            then
                echo "Sourcing quick cached underlay."
                source "{options.docker_entrypoint_sh_underlay_sh_container_path}"
            elif [[ -f "{options.docker_entrypoint_sh_setup_underlay_container_path}" ]]
            then
                echo "Sourcing underlay setup from ROS install."
                source "{options.docker_entrypoint_sh_setup_underlay_container_path}"
                if [[ -d "{options.docker_entrypoint_sh_underlay_sh_container_path.parent}" ]]
                then
                    echo "Caching underlay to enable quick setup."
                    declare -px > {options.docker_entrypoint_sh_underlay_sh_container_path}
                fi
            fi

            docker_entrypoint_sh_quick_overlay="{options.docker_entrypoint_sh_quick_overlay}"
            docker_entrypoint_sh_setup_overlay="{options.docker_entrypoint_sh_setup_overlay}"

            if [[ $docker_entrypoint_sh_setup_overlay = "False" ]]
            then
                echo "Not sourcing overlay from workspace install."
            elif
                [[ $docker_entrypoint_sh_quick_overlay = "True" ]] && \
                [[ -f "{options.docker_entrypoint_sh_overlay_sh_container_path}" ]] && \
                [[ {options.docker_entrypoint_sh_overlay_sh_container_path} -nt \
                   {options.docker_entrypoint_sh_setup_overlay_container_path} ]]
            then
                echo "Sourcing quick cached overlay from workspace install."
                source "{options.docker_entrypoint_sh_overlay_sh_container_path}"
            elif [[ -f "{options.docker_entrypoint_sh_setup_overlay_container_path}" ]]
            then
                echo "Sourcing overlay setup from workspace install."
                source "{options.docker_entrypoint_sh_setup_overlay_container_path}"
                if [[ -d "{options.docker_entrypoint_sh_overlay_sh_container_path.parent}" ]]
                then
                    echo "Caching overlay to enable quick setup."
                    declare -px > {options.docker_entrypoint_sh_overlay_sh_container_path}
                fi
            fi

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
