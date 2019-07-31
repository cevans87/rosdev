from dataclasses import dataclass, field, replace
from frozendict import frozendict
from logging import getLogger
from pathlib import Path
from textwrap import dedent
from typing import Tuple, Type

from rosdev.gen.home import GenHome
from rosdev.gen.ros.overlay.setup_bash import GenRosOverlaySetupBash
from rosdev.gen.ros.underlay.setup_bash import GenRosUnderlaySetupBash
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.subprocess import exec


log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerBashrc(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenHome,
        GenRosOverlaySetupBash,
        GenRosUnderlaySetupBash,
    ))

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        docker_bashrc_container_path = options.resolve_path(
            options.docker_bashrc_container_path
        )

        docker_bashrc_ln_target_path = options.resolve_path(
            options.docker_bashrc_ln_target_path
        )

        docker_bashrc_workspace_path = options.resolve_path(
            options.docker_bashrc_workspace_path
        )

        docker_container_volumes = dict(options.docker_container_volumes)
        docker_container_volumes[docker_bashrc_workspace_path] = docker_bashrc_container_path
        docker_container_volumes = frozendict(docker_container_volumes)

        return replace(
            options,
            docker_container_volumes=docker_container_volumes,
            docker_bashrc_container_path=docker_bashrc_container_path,
            docker_bashrc_ln_target_path=docker_bashrc_ln_target_path,
            docker_bashrc_workspace_path=docker_bashrc_workspace_path,
        )

    @classmethod
    def get_docker_bashrc_contents(cls, options: Options) -> str:
        return dedent(r'''
            # If this is an xterm set the title to user@host:dir
            case "$TERM" in
            xterm*|rxvt*)
                PROMPT_COMMAND='echo -ne "\033]0;${USER}@${HOSTNAME}: ${PWD}\007"'
                ;;
            *)
                ;;
            esac
            
            # enable bash completion in interactive shells
            if ! shopt -oq posix; then
              if [ -f /usr/share/bash-completion/bash_completion ]; then
                . /usr/share/bash-completion/bash_completion
              elif [ -f /etc/bash_completion ]; then
                . /etc/bash_completion
              fi
            fi
        ''').lstrip()

    @classmethod
    async def main(cls, options: Options) -> None:
        log.info(
            f'Creating docker container bashrc at {options.docker_bashrc_workspace_path}'
        )
        await exec(f'mkdir -p {options.docker_bashrc_workspace_path.parent}')
        with open(Path(options.docker_bashrc_workspace_path), 'w') as f_out:
            f_out.write(cls.get_docker_bashrc_contents(options))
        await exec(
            f'docker exec {options.docker_container_name} '
            f'ln -s {options.docker_bashrc_container_path} {options.docker_bashrc_ln_target_path}'
        )

        log.info(
            f'Created docker container bashrc at {options.docker_bashrc_workspace_path}'
        )
