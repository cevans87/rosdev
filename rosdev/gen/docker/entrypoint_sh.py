from dataclasses import dataclass, field, replace
from frozendict import frozendict
from logging import getLogger, INFO
from textwrap import dedent
from typing import Tuple, Type

from rosdev.gen.host import GenHost
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerEntrypointSh(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenHost,
    ))
        
    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        docker_container_environment = dict(Options.docker_container_environment)
        if options.docker_entrypoint_sh_quick_setup_overlay:
            docker_container_environment[
                options.docker_entrypoint_sh_quick_setup_overlay_path_env_name
            ] = f'{options.docker_entrypoint_sh_quick_setup_overlay_path}'
            docker_container_environment[
                options.docker_entrypoint_sh_quick_setup_overlay_path_parent_env_name
            ] = f'{options.docker_entrypoint_sh_quick_setup_overlay_path.parent}'
        if options.docker_entrypoint_sh_quick_setup_underlay:
            docker_container_environment[
                options.docker_entrypoint_sh_quick_setup_underlay_path_env_name
            ] = f'{options.docker_entrypoint_sh_quick_setup_underlay_path}'
            docker_container_environment[
                options.docker_entrypoint_sh_quick_setup_underlay_path_parent_env_name
            ] = f'{options.docker_entrypoint_sh_quick_setup_underlay_path.parent}'
        if options.docker_entrypoint_sh_setup_overlay:
            docker_container_environment[
                options.docker_entrypoint_sh_setup_overlay_path_env_name
            ] = f'{options.docker_entrypoint_sh_setup_overlay_path}'
        if options.docker_entrypoint_sh_setup_underlay:
            docker_container_environment[
                options.docker_entrypoint_sh_setup_underlay_path_env_name
            ] = f'{options.docker_entrypoint_sh_setup_underlay_path}'

        docker_container_environment = frozendict(docker_container_environment)

        docker_container_volumes = dict(options.docker_container_volumes)
        docker_container_volumes[options.docker_entrypoint_sh_host_path] = (
            options.docker_entrypoint_sh_container_path
        )
        docker_container_volumes = frozendict(docker_container_volumes)

        return replace(
            options,
            docker_container_environment=docker_container_environment,
            docker_container_volumes=docker_container_volumes,
        )

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        log.debug(f'{options.docker_entrypoint_sh_quick_setup_overlay = }')
        log.debug(f'{options.docker_entrypoint_sh_quick_setup_underlay = }')
        log.debug(f'{options.docker_entrypoint_sh_setup_overlay = }')
        log.debug(f'{options.docker_entrypoint_sh_setup_overlay_path = }')
        log.debug(f'{options.docker_entrypoint_sh_setup_underlay = }')
        log.debug(f'{options.docker_entrypoint_sh_setup_underlay_path = }')

        log.debug(f'{options.docker_entrypoint_sh_container_path = }')
        log.debug(f'{options.docker_entrypoint_sh_host_path = }')

    @classmethod
    def get_docker_entrypoint_sh_contents(cls, options: Options) -> str:
        return dedent(fr'''
            #!/bin/bash
            set -e

            if 
                [[ 
                    -z ${{{options.docker_entrypoint_sh_setup_underlay_path_env_name}}}
                ]] || [[ 
                    ! -f ${{{options.docker_entrypoint_sh_setup_underlay_path_env_name}}}
                ]]
            then
                if
                    [[
                        ! -z ${{{options.docker_entrypoint_sh_log_level_env_name}}}
                    ]] && [[
                        ${{{options.docker_entrypoint_sh_log_level_env_name}}} -le {INFO}
                    ]]
                then
                    echo "Not sourcing underlay from ROS install."
                fi
            elif
                [[
                    ! -z ${{{options.docker_entrypoint_sh_quick_setup_underlay_path_env_name}}}
                ]] && [[ 
                    -f ${{{options.docker_entrypoint_sh_quick_setup_underlay_path_env_name}}}
                ]] && [[  
                    ${{{options.docker_entrypoint_sh_quick_setup_underlay_path_env_name}}} -nt \
                    ${{{options.docker_entrypoint_sh_setup_underlay_path_env_name}}}
                ]]
            then
                if
                    [[
                        ! -z ${{{options.docker_entrypoint_sh_log_level_env_name}}}
                    ]] && [[
                        ${{{options.docker_entrypoint_sh_log_level_env_name}}} -le {INFO}
                    ]]
                then
                    echo "Sourcing cached quick setup underlay."
                fi
                source ${{{options.docker_entrypoint_sh_quick_setup_underlay_path_env_name}}}
            else
                if
                    [[
                        ! -z ${{{options.docker_entrypoint_sh_log_level_env_name}}}
                    ]] && [[
                        ${{{options.docker_entrypoint_sh_log_level_env_name}}} -le {INFO}
                    ]]
                then
                    echo "Sourcing setup underlay."
                fi
                source ${{{options.docker_entrypoint_sh_setup_underlay_path_env_name}}}
                if
                    [[ 
                        ! -z ${{{options.docker_entrypoint_sh_quick_setup_underlay_path_parent_env_name}}}
                    ]] && [[ 
                        -d ${{{options.docker_entrypoint_sh_quick_setup_underlay_path_parent_env_name}}}
                    ]]
                then
                    if
                        [[
                            ! -z ${{{options.docker_entrypoint_sh_log_level_env_name}}}
                        ]] && [[
                            ${{{options.docker_entrypoint_sh_log_level_env_name}}} -le {INFO}
                        ]]
                    then
                        echo "Caching underlay to enable quick setup."
                    fi
                    declare -px > ${{{options.docker_entrypoint_sh_quick_setup_underlay_path_env_name}}}
                fi
            fi

            if 
                [[
                    -z ${{{options.docker_entrypoint_sh_setup_overlay_path_env_name}}}
                ]] || [[
                    ! -f ${{{options.docker_entrypoint_sh_setup_overlay_path_env_name}}}
                ]]
            then
                if
                    [[
                        ! -z ${{{options.docker_entrypoint_sh_log_level_env_name}}}
                    ]] && [[
                        ${{{options.docker_entrypoint_sh_log_level_env_name}}} -le {INFO}
                    ]]
                then
                    echo "Not sourcing overlay from ROS install."
                fi
            elif
                [[
                    -z ${{{options.docker_entrypoint_sh_quick_setup_overlay_path_env_name}}}
                ]] && [[
                    -f ${{{options.docker_entrypoint_sh_quick_setup_overlay_path_env_name}}}
                ]] && [[ 
                    ${{{options.docker_entrypoint_sh_quick_setup_overlay_path_env_name}}} -nt \
                    ${{{options.docker_entrypoint_sh_setup_overlay_path_env_name}}}
                ]]
            then
                if
                    [[
                        ! -z ${{{options.docker_entrypoint_sh_log_level_env_name}}}
                    ]] && [[
                        ${{{options.docker_entrypoint_sh_log_level_env_name}}} -le {INFO}
                    ]]
                then
                    echo "Sourcing cached quick setup overlay."
                fi
                source ${{{options.docker_entrypoint_sh_quick_setup_overlay_path_env_name}}}
            else
                if
                    [[
                        ! -z ${{{options.docker_entrypoint_sh_log_level_env_name}}}
                    ]] && [[
                        ${{{options.docker_entrypoint_sh_log_level_env_name}}} -le {INFO}
                    ]]
                then
                    echo "Sourcing setup overlay."
                fi
                source ${{{options.docker_entrypoint_sh_setup_overlay_path_env_name}}}
                if
                    [[
                        ! -z ${{{options.docker_entrypoint_sh_quick_setup_overlay_path_parent_env_name}}}
                    ]] && [[ 
                        -d ${{{options.docker_entrypoint_sh_quick_setup_overlay_path_parent_env_name}}}
                    ]]
                then
                    if
                        [[
                            ! -z ${{{options.docker_entrypoint_sh_log_level_env_name}}}
                        ]] && [[
                            ${{{options.docker_entrypoint_sh_log_level_env_name}}} -le {INFO}
                        ]]
                    then
                        echo "Caching overlay to enable quick setup."
                    fi
                    declare -px > ${{{options.docker_entrypoint_sh_quick_setup_overlay_path_env_name}}}
                fi
            fi

            unset "${{!ROSDEV_GEN_DOCKER_ENTRYPOINT_SH@}}"

            exec "$@"
        ''').lstrip()

    @classmethod
    async def main(cls, options: Options) -> None:
        log.info(f'Creating docker_container_entrypoint_sh')

        GenHost.write_text(
            data=cls.get_docker_entrypoint_sh_contents(options),
            options=options,
            path=options.docker_entrypoint_sh_host_path,
        )
        # Make executable
        options.docker_entrypoint_sh_host_path.chmod(0o755)

        log.info(f'Created docker_container_entrypoint_sh')
