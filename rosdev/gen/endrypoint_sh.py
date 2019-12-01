from atools import memoize
from dataclasses import dataclass
from frozendict import frozendict
from logging import getLogger, INFO
from textwrap import dedent
from typing import Dict, Mapping

from rosdev.gen.host import GenHost
from rosdev.gen.install_base import GenInstallBase
from rosdev.gen.rosdev.home import GenRosdevHome
from rosdev.gen.rosdev.workspace import GenRosdevWorkspace
from rosdev.gen.workspace import GenWorkspace
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.path import Path

log = getLogger(__name__)


@dataclass(frozen=True)
class GenEntrypointSh(Handler):

    @staticmethod
    @memoize
    async def get_environment(options: Options) -> Mapping[str, str]:
        environment: Dict[str, str] = {}
        if options.entrypoint_sh_quick_setup_overlay:
            environment[
                await GenEntrypointSh.get_quick_setup_overlay_path_env_name(options)
            ] = (
                f'{await GenEntrypointSh.get_quick_setup_overlay_path(options)}'
            )
            environment[
                await GenEntrypointSh.get_quick_setup_overlay_path_parent_env_name(options)
            ] = (
                f'{(await GenEntrypointSh.get_quick_setup_overlay_path(options)).parent}'
            )
        if options.entrypoint_sh_quick_setup_underlay:
            environment[
                await GenEntrypointSh.get_quick_setup_underlay_path_env_name(options)
            ] = (
                f'{await GenEntrypointSh.get_quick_setup_underlay_path(options)}'
            )
            environment[
                await GenEntrypointSh.get_quick_setup_underlay_path_parent_env_name(options)
            ] = (
                f'{(await GenEntrypointSh.get_quick_setup_underlay_path(options)).parent}'
            )
        if options.entrypoint_sh_setup_overlay:
            environment[
                await GenEntrypointSh.get_setup_overlay_path_env_name(options)
            ] = (
                f'{await GenEntrypointSh.get_setup_overlay_path(options)}'
            )
        if options.entrypoint_sh_setup_underlay:
            environment[
                await GenEntrypointSh.get_setup_underlay_path_env_name(options)
            ] = (
                f'{await GenEntrypointSh.get_setup_underlay_path(options)}'
            )

        environment: Mapping[str, str] = frozendict(environment)

        log.debug(f'{GenEntrypointSh.__name__} {environment = }')

        return environment

    @staticmethod
    @memoize
    async def get_environment_flags(options: Options) -> str:
        environment_flags = ' '.join([
            f'-e {k}={v}'
            for k, v in (await GenEntrypointSh.get_environment(options)).items()
        ])

        log.debug(f'{GenEntrypointSh.__name__} {environment_flags = }')

        return environment_flags

    # noinspection PyUnusedLocal
    @staticmethod
    @memoize
    async def get_container_path(options: Options) -> Path:
        container_path = Path('/') / 'rosdev_entrypoint.sh'

        log.debug(f'{GenEntrypointSh.__name__} {container_path = }')

        return container_path

    @staticmethod
    @memoize
    async def get_path(options: Options) -> Path:
        path = await GenRosdevHome.get_path(options) / '' / 'rosdev_entrypoint.sh'

        log.debug(f'{GenEntrypointSh.__name__} {path = }')

        return path

    # noinspection PyUnusedLocal
    @staticmethod
    @memoize
    async def get_log_level_env_name(options: Options) -> str:
        log_level_env_name = 'ROSDEV_GEN_ENTRYPOINT_LOG_LEVEL'

        log.debug(f'{GenEntrypointSh.__name__} {log_level_env_name = }')

        return log_level_env_name

    # noinspection PyUnusedLocal
    @staticmethod
    @memoize
    async def get_path_env_name(options: Options) -> str:
        path_env_name = 'ROSDEV_GEN_ENTRYPOINT_SH_PATH'

        log.debug(f'{GenEntrypointSh.__name__} {path_env_name = }')

        return path_env_name

    @staticmethod
    @memoize
    async def get_quick_setup_overlay_path(options: Options) -> Path:
        quick_setup_overlay_path = (
                await GenRosdevWorkspace.get_path(options) / 'quick_setup_overlay.sh'
        )

        log.debug(f'{GenEntrypointSh.__name__} {quick_setup_overlay_path = }')

        return quick_setup_overlay_path

    # noinspection PyUnusedLocal
    @staticmethod
    @memoize
    async def get_quick_setup_overlay_path_env_name(options: Options) -> str:
        quick_setup_overlay_path_env_name = (
            'ROSDEV_GEN_ENTRYPOINT_SH_QUICK_SETUP_OVERLAY_PATH'
        )

        log.debug(f'{GenEntrypointSh.__name__} {quick_setup_overlay_path_env_name = }')

        return quick_setup_overlay_path_env_name

    # noinspection PyUnusedLocal
    @staticmethod
    @memoize
    async def get_quick_setup_overlay_path_parent_env_name(options: Options) -> str:
        quick_setup_overlay_path_parent_env_name = (
            'ROSDEV_GEN_ENTRYPOINT_SH_QUICK_SETUP_OVERLAY_PATH_PARENT'
        )

        log.debug(f'{GenEntrypointSh.__name__} {quick_setup_overlay_path_parent_env_name = }')

        return quick_setup_overlay_path_parent_env_name

    @staticmethod
    @memoize
    async def get_quick_setup_underlay_path(options: Options) -> Path:
        quick_setup_underlay_path = (
                await GenRosdevWorkspace.get_path(options) / 'quick_setup_underlay.sh'
        )

        log.debug(f'{GenEntrypointSh.__name__} {quick_setup_underlay_path = }')

        return quick_setup_underlay_path

    # noinspection PyUnusedLocal
    @staticmethod
    @memoize
    async def get_quick_setup_underlay_path_env_name(options: Options) -> str:
        quick_setup_underlay_path_env_name = (
            'ROSDEV_GEN_ENTRYPOINT_SH_QUICK_SETUP_UNDERLAY_PATH'
        )

        log.debug(f'{GenEntrypointSh.__name__} {quick_setup_underlay_path_env_name = }')

        return quick_setup_underlay_path_env_name

    # noinspection PyUnusedLocal
    @staticmethod
    @memoize
    async def get_quick_setup_underlay_path_parent_env_name(options: Options) -> str:
        quick_setup_underlay_path_parent_env_name = (
            'ROSDEV_GEN_ENTRYPOINT_SH_QUICK_SETUP_UNDERLAY_PATH_PARENT'
        )

        log.debug(
            f'{GenEntrypointSh.__name__} {quick_setup_underlay_path_parent_env_name = }'
        )

        return quick_setup_underlay_path_parent_env_name

    @staticmethod
    @memoize
    async def get_setup_overlay_path(options: Options) -> Path:
        setup_overlay_path = await GenWorkspace.get_path(options) / 'install' / 'setup.bash'

        log.debug(f'{GenEntrypointSh.__name__} {setup_overlay_path = }')

        return setup_overlay_path

    # noinspection PyUnusedLocal
    @staticmethod
    @memoize
    async def get_setup_overlay_path_env_name(options: Options) -> str:
        setup_overlay_path_env_name = 'ROSDEV_GEN_ENTRYPOINT_SH_SETUP_OVERLAY_PATH'

        log.debug(f'{GenEntrypointSh.__name__} {setup_overlay_path_env_name = }')

        return setup_overlay_path_env_name

    @staticmethod
    @memoize
    async def get_setup_underlay_path(options: Options) -> Path:
        setup_underlay_path = await GenInstallBase.get_path(options) / 'setup.bash'

        log.debug(f'{GenEntrypointSh.__name__} {setup_underlay_path = }')

        return setup_underlay_path

    # noinspection PyUnusedLocal
    @staticmethod
    @memoize
    async def get_setup_underlay_path_env_name(options: Options) -> str:
        setup_underlay_path_env_name = 'ROSDEV_GEN_ENTRYPOINT_SH_SETUP_UNDERLAY_PATH'

        log.debug(f'{GenEntrypointSh.__name__} {setup_underlay_path_env_name = }')

        return setup_underlay_path_env_name

    @staticmethod
    @memoize
    async def get_overlay_path(options: Options) -> Path:
        overlay_path = await GenRosdevWorkspace.get_path(options) / 'overlay.sh'

        log.debug(f'{GenEntrypointSh.__name__} {overlay_path = }')

        return overlay_path

    @staticmethod
    @memoize
    async def get_underlay_path(options: Options) -> Path:
        underlay_path = await GenRosdevWorkspace.get_path(options) / 'underlay.sh'

        log.debug(f'{GenEntrypointSh.__name__} {underlay_path = }')

        return underlay_path

    @staticmethod
    @memoize
    async def get_text(options: Options) -> str:
        # noinspection PyPep8
        return dedent(fr'''
            #!/bin/bash
            set -e

            if 
                [[ 
                    -z ${{{await GenEntrypointSh.get_setup_underlay_path_env_name(options)}}}
                ]] || [[ 
                    ! -f ${{{await GenEntrypointSh.get_setup_underlay_path_env_name(options)}}}
                ]]
            then
                if
                    [[
                        ! -z ${{{await GenEntrypointSh.get_log_level_env_name(options)}}}
                    ]] && [[
                        ${{{await GenEntrypointSh.get_log_level_env_name(options)}}} -le {INFO}
                    ]]
                then
                    echo "Not sourcing underlay from ROS install."
                fi
            elif
                [[
                    ! -z ${{{await GenEntrypointSh.get_quick_setup_underlay_path_env_name(options)}}}
                ]] && [[ 
                    -f ${{{await GenEntrypointSh.get_quick_setup_underlay_path_env_name(options)}}}
                ]] && [[  
                    ${{{await GenEntrypointSh.get_quick_setup_underlay_path_env_name(options)}}} -nt \
                    ${{{await GenEntrypointSh.get_setup_underlay_path_env_name(options)}}}
                ]]
            then
                if
                    [[
                        ! -z ${{{await GenEntrypointSh.get_log_level_env_name(options)}}}
                    ]] && [[
                        ${{{await GenEntrypointSh.get_log_level_env_name(options)}}} -le {INFO}
                    ]]
                then
                    echo "Sourcing cached quick setup underlay."
                fi
                source ${{{await GenEntrypointSh.get_quick_setup_underlay_path_env_name(options)}}}
            else
                if
                    [[
                        ! -z ${{{await GenEntrypointSh.get_log_level_env_name(options)}}}
                    ]] && [[
                        ${{{await GenEntrypointSh.get_log_level_env_name(options)}}} -le {INFO}
                    ]]
                then
                    echo "Sourcing setup underlay."
                fi
                source ${{{await GenEntrypointSh.get_setup_underlay_path_env_name(options)}}}
                if
                    [[ 
                        ! -z ${{{await GenEntrypointSh.get_quick_setup_underlay_path_parent_env_name(options)}}}
                    ]] && [[ 
                        -d ${{{await GenEntrypointSh.get_quick_setup_underlay_path_parent_env_name(options)}}}
                    ]]
                then
                    if
                        [[
                            ! -z ${{{await GenEntrypointSh.get_log_level_env_name(options)}}}
                        ]] && [[
                            ${{{await GenEntrypointSh.get_log_level_env_name(options)}}} -le {INFO}
                        ]]
                    then
                        echo "Caching underlay to enable quick setup."
                    fi
                    declare -px > ${{{await GenEntrypointSh.get_quick_setup_underlay_path_env_name(options)}}}
                fi
            fi

            if 
                [[
                    -z ${{{await GenEntrypointSh.get_setup_overlay_path_env_name(options)}}}
                ]] || [[
                    ! -f ${{{await GenEntrypointSh.get_setup_overlay_path_env_name(options)}}}
                ]]
            then
                if
                    [[
                        ! -z ${{{await GenEntrypointSh.get_log_level_env_name(options)}}}
                    ]] && [[
                        ${{{await GenEntrypointSh.get_log_level_env_name(options)}}} -le {INFO}
                    ]]
                then
                    echo "Not sourcing overlay from ROS install."
                fi
            elif
                [[
                    -z ${{{await GenEntrypointSh.get_quick_setup_overlay_path_env_name(options)}}}
                ]] && [[
                    -f ${{{await GenEntrypointSh.get_quick_setup_overlay_path_env_name(options)}}}
                ]] && [[ 
                    ${{{await GenEntrypointSh.get_quick_setup_overlay_path_env_name(options)}}} -nt \
                    ${{{await GenEntrypointSh.get_setup_overlay_path_env_name(options)}}}
                ]]
            then
                if
                    [[
                        ! -z ${{{await GenEntrypointSh.get_log_level_env_name(options)}}}
                    ]] && [[
                        ${{{await GenEntrypointSh.get_log_level_env_name(options)}}} -le {INFO}
                    ]]
                then
                    echo "Sourcing cached quick setup overlay."
                fi
                source ${{{await GenEntrypointSh.get_quick_setup_overlay_path_env_name(options)}}}
            else
                if
                    [[
                        ! -z ${{{await GenEntrypointSh.get_log_level_env_name(options)}}}
                    ]] && [[
                        ${{{await GenEntrypointSh.get_log_level_env_name(options)}}} -le {INFO}
                    ]]
                then
                    echo "Sourcing setup overlay."
                fi
                source ${{{await GenEntrypointSh.get_setup_overlay_path_env_name(options)}}}
                if
                    [[
                        ! -z ${{{await GenEntrypointSh.get_quick_setup_overlay_path_parent_env_name(options)}}}
                    ]] && [[ 
                        -d ${{{await GenEntrypointSh.get_quick_setup_overlay_path_parent_env_name(options)}}}
                    ]]
                then
                    if
                        [[
                            ! -z ${{{await GenEntrypointSh.get_log_level_env_name(options)}}}
                        ]] && [[
                            ${{{await GenEntrypointSh.get_log_level_env_name(options)}}} -le {INFO}
                        ]]
                    then
                        echo "Caching overlay to enable quick setup."
                    fi
                    declare -px > ${{{await GenEntrypointSh.get_quick_setup_overlay_path_env_name(options)}}}
                fi
            fi

            unset "${{!ROSDEV_GEN_ENTRYPOINT_SH@}}"

            exec "$@"
        ''').lstrip()

    @classmethod
    async def main(cls, options: Options) -> None:
        if (await cls.get_path(options)).exists():
            return

        log.info(f'Creating container_entrypoint_sh')

        GenHost.write_text(
            data=await cls.get_text(options),
            options=options,
            path=await cls.get_path(options),
        )
        # Make executable
        (await cls.get_path(options)).chmod(0o755)

        log.info(f'Created container_entrypoint_sh')
