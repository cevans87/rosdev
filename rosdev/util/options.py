from dataclasses import dataclass
from frozendict import frozendict
from logging import getLogger
from platform import machine
from typing import Mapping, Tuple


log = getLogger(__name__)


@dataclass(frozen=True)
class Options:
    architecture: str = {'x86_64': 'amd64', 'arm': 'arm32v7', 'aarch64': 'arm64v8'}[machine()]
    backend_ssh_builder_identity_path: str = ''
    backend_ssh_builder_uri: str = ''
    backend_ssh_runner_identity_path: str = ''
    backend_ssh_runner_uri: str = ''
    build_type: str = 'Debug'
    colcon_build_args: str = ''
    docker_container_ccache: bool = True
    docker_container_gui: bool = False
    docker_container_ports: Mapping[int, int] = frozendict()  # container:host
    docker_container_replace: bool = False
    docker_container_volumes: Mapping[str, str] = frozendict()  # host:container
    docker_entrypoint_sh_quick_setup_overlay: bool = True
    docker_entrypoint_sh_quick_setup_underlay: bool = True
    docker_entrypoint_sh_setup_overlay: bool = True
    docker_entrypoint_sh_setup_underlay: bool = True
    docker_image_pull: bool = False
    docker_image_replace: bool = False
    dry_run: bool = False
    executable: str = ''
    idea_ide_name: str = ''
    idea_ide_path: str = ''
    idea_uuid: str = ''
    log_level: str = 'INFO'
    package: str = ''
    release: str = 'latest'
    remainder: Tuple[str] = tuple()
    rosdep_install_args: str = ''
    run_main: bool = True
    run_validate_options: bool = True
    sanitizer: str = ''

    def __post_init__(self) -> None:
        assert not ((self.architecture != 'amd64') and (self.release == 'latest')), (
            f'Not supported: {self.architecture = }, {self.release = }'
        )
