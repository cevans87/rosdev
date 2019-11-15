from dataclasses import dataclass
from frozendict import frozendict
from logging import getLogger
from pathlib import Path
from platform import machine
from typing import Mapping, Optional
from uuid import UUID


log = getLogger(__name__)


@dataclass(frozen=True)
class Options:
    architecture: str = {'x86_64': 'amd64', 'arm': 'arm32v7', 'aarch64': 'arm64v8'}[machine()]

    build_type: str = 'Debug'

    colcon_build_args: Optional[str] = None

    docker_container_ccache: bool = True
    docker_container_gui: bool = False
    docker_container_ports: Mapping[int, Optional[int]] = frozendict()  # container:host
    docker_container_replace: bool = False
    docker_container_volumes: Mapping[Path, Path] = frozendict()  # host:container

    docker_entrypoint_sh_quick_setup_overlay: bool = True
    docker_entrypoint_sh_quick_setup_underlay: bool = True
    docker_entrypoint_sh_setup_overlay: bool = True
    docker_entrypoint_sh_setup_underlay: bool = True

    docker_image_pull: bool = False
    docker_image_replace: bool = False

    dry_run: bool = False

    executable: Optional[str] = None

    idea_ide_name: Optional[str] = None
    idea_ide_path: Optional[Path] = None

    idea_uuid: Optional[UUID] = None
    log_level: str = 'INFO'
    package: Optional[str] = None
    pull_build: bool = False
    release: str = 'latest'
    rosdep_install_args: Optional[str] = None
    run_main: bool = True
    run_validate_options: bool = True
    sanitizer: Optional[str] = None

    @property
    def idea_clion_cpp_toolchains_xml_path(self) -> Path:
        return self.idea_home_path / 'options' / 'cpp.toolchains.xml'

    @property
    def idea_clion_iml_path(self) -> Path:
        return self.idea_workspace_path / f'{self.workspace_path.parts[-1]}.iml'
    
    @property
    def idea_clion_misc_xml_path(self) -> Path:
        return self.idea_workspace_path / 'misc.xml'

    @property
    def idea_clion_webservers_xml_path(self) -> Path:
        return self.idea_home_path / 'options' / 'webServers.xml'

    @property
    def idea_clion_deployment_xml_path(self) -> Path:
        return self.idea_workspace_path / 'deployment.xml'

    @property
    def idea_clion_webservers_name(self) -> str:
        return f'rosdev_{self.idea_ide_name}_{self.workspace_hash} ({self.idea_uuid})'

    @property
    def idea_clion_workspace_xml_path(self) -> Path:
        return self.idea_workspace_path / 'workspace.xml'

    @property
    def machine(self) -> str:
        return {
            'amd64': 'x86_64',
            'arm32v7': 'armhf',
            'arm64v8': 'aarch64'
        }[self.architecture]

    @property
    def operating_system(self) -> str:
        return {
            'amd64': 'linux',
            'arm32v7': f'linux-armhf',
            'arm64v8': f'linux-aarch64',
        }[self.architecture]

    def __post_init__(self) -> None:
        assert not ((self.architecture != 'amd64') and (self.release == 'latest')), (
            f'Not supported: {self.architecture = }, {self.release = }'
        )
