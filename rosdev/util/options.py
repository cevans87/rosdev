from dataclasses import dataclass
from frozendict import frozendict
from logging import getLogger
import os
from pathlib import Path
from typing import Mapping, Optional
from uuid import UUID


log = getLogger(__name__)


@dataclass(frozen=True)
class Options:
    architecture: Optional[str] = None
    build_type: str = 'Debug'

    colcon_build_args: Optional[str] = None

    docker_container_ccache: bool = True
    docker_container_gui: bool = False
    docker_container_environment: Mapping[str, str] = frozendict()
    docker_container_ports: Mapping[int, int] = frozendict()  # container:host
    docker_container_replace: bool = False
    docker_container_volumes: Mapping[Path, Path] = frozendict()  # host:container

    docker_entrypoint_sh_quick_setup_overlay: bool = True
    docker_entrypoint_sh_quick_setup_underlay: bool = True
    docker_entrypoint_sh_setup_overlay: bool = True
    docker_entrypoint_sh_setup_underlay: bool = True

    docker_image_base_tag: Optional[str] = None

    docker_ssh_port: Optional[int] = None

    executable: Optional[str] = None

    idea_ide_name: Optional[str] = None
    idea_ide_start_path: Optional[Path] = None
    idea_home_path: Optional[Path] = None

    idea_uuid: Optional[UUID] = None
    log_level: str = 'INFO'
    package: Optional[str] = None
    pull_build: bool = False
    pull_docker_image: bool = False
    release: str = 'latest'
    rosdep_install_args: Optional[str] = None
    run_main: bool = True
    run_validate_options: bool = True
    sanitizer: Optional[str] = None

    reuse_environment: bool = True
    reuse_docker_ssh_pam_environment: bool = True

    stage: Optional[str] = None

    workspace_hash: Optional[str] = None
    workspace_path: Optional[Path] = None
    
    @property
    def home_path(self) -> Path:
        return Path.home()

    @property
    def docker_image_tag(self) -> str:
        return f'rosdev:{self.release}_{self.architecture}'

    @property
    def docker_container_name(self) -> str:
        return f'rosdev_{self.release}_{self.architecture}_{self.workspace_hash}'

    @property
    def docker_ssh_port_path(self) -> Path:
        return self.workspace_rosdev_path / 'docker_ssh_port'

    @property
    def idea_workspace_path(self) -> Path:
        return self.workspace_path / '.idea'

    @property
    def idea_c_kdbx_path(self) -> Path:
        return self.idea_home_path / 'c.kdbx'
    
    @property
    def idea_c_pwd_path(self) -> Path:
        return self.idea_home_path / 'c.pwd'
    
    @property
    def idea_clion_cpp_toolchains_xml_path(self) -> Path:
        return self.idea_home_path / 'options' / 'cpp.toolchains.xml'

    @property
    def idea_clion_webservers_xml_path(self) -> Path:
        return self.idea_home_path / 'options' / 'webServers.xml'

    @property
    def idea_pycharm_jdk_table_xml_sftp_uri(self) -> str:
        return f'sftp://{os.getlogin()}@localhost:{self.docker_ssh_port}/usr/bin/python'

    @property
    def idea_pycharm_jdk_table_xml_ssh_uri(self) -> str:
        return f'ssh://{os.getlogin()}@localhost:{self.docker_ssh_port}/usr/bin/python'
    
    @property
    def idea_pycharm_misc_xml_path(self) -> Path:
        return self.idea_workspace_path / 'misc.xml'

    @property
    def idea_pycharm_webservers_xml_path(self) -> Path:
        return self.idea_workspace_path / 'webServers.xml'

    @property
    def idea_clion_deployment_xml_path(self) -> Path:
        return self.idea_workspace_path / 'deployment.xml'

    @property
    def idea_pycharm_deployment_xml_path(self) -> Path:
        return self.idea_workspace_path / 'deployment.xml'

    @property
    def idea_pycharm_jdk_table_xml_path(self) -> Path:
        return self.idea_home_path / 'options' / 'jdk.table.xml'

    @property
    def idea_workspace_xml_path(self) -> Path:
        return self.idea_workspace_path / 'workspace.xml'
    
    @property
    def idea_security_xml_path(self) -> Path:
        return self.idea_home_path / 'options' / 'security.xml'

    @property
    def idea_clion_webservers_name(self) -> str:
        return f'rosdev_{self.idea_ide_name}_{self.workspace_hash} ({self.idea_uuid})'

    @property
    def idea_pycharm_webservers_name(self) -> str:
        return f'rosdev_{self.idea_ide_name}_{self.workspace_hash}'
    
    @property
    def idea_uuid_path(self) -> Path:
        return self.workspace_rosdev_path / 'idea_uuid'

    @property
    def idea_webservers_xml_path(self) -> Path:
        return self.idea_workspace_path / 'webServers.xml'

    @property
    def docker_gdbinit_symlink_container_path(self) -> Path:
        return self.home_path / '.gdbinit'

    @property
    def docker_gdbinit_path(self) -> Path:
        return self.workspace_rosdev_path / 'gdbinit'

    @property
    def home_rosdev_path(self) -> Path:
        return self.home_path / '.rosdev' / self.release / self.architecture

    @property
    def workspace_rosdev_path(self) -> Path:
        return self.workspace_path / '.rosdev' / self.release / self.architecture

    @property
    def install_id_path(self) -> Path:
        return self.home_rosdev_path / 'install_id'

    @property
    def install_path(self) -> Path:
        return self.home_rosdev_path / 'install'

    @property
    def install_symlink_path(self) -> Path: 
        return self.workspace_rosdev_path / 'install'

    @property
    def src_id_path(self) -> Path:
        return self.home_rosdev_path / 'src_id'
    
    @property
    def src_path(self) -> Path:
        return self.home_rosdev_path / 'src'

    @property
    def src_symlink_path(self) -> Path:
        return self.workspace_rosdev_path / 'src'
    
    @property
    def overrides_path(self) -> Path:
        return self.workspace_path / '.rosdev' / 'overrides'
    
    @property
    def docker_path(self) -> Path:
        return self.home_rosdev_path / 'docker'

    @property
    def docker_dockerfile_path(self) -> Path:
        return self.docker_path / 'Dockerfile'

    @property
    def docker_entrypoint_sh_container_path(self) -> Path:
        return Path('/') / 'rosdev_docker_entrypoint.sh'

    @property
    def docker_entrypoint_sh_host_path(self) -> Path:
        return self.docker_path / 'rosdev_docker_entrypoint.sh'
    
    @property
    def docker_entrypoint_sh_path_env_name(self) -> str:
        return 'ROSDEV_GEN_DOCKER_ENTRYPOINT_SH_PATH'

    @property
    def docker_entrypoint_sh_quick_setup_overlay_path(self) -> Path:
        return self.workspace_rosdev_path / 'quick_setup_overlay.sh'

    @property
    def docker_entrypoint_sh_quick_setup_overlay_path_env_name(self) -> str:
        return 'ROSDEV_GEN_DOCKER_ENTRYPOINT_SH_QUICK_SETUP_OVERLAY_PATH'

    @property
    def docker_entrypoint_sh_quick_setup_overlay_path_parent_env_name(self) -> str:
        return 'ROSDEV_GEN_DOCKER_ENTRYPOINT_SH_QUICK_SETUP_OVERLAY_PATH_PARENT'

    @property
    def docker_entrypoint_sh_quick_setup_underlay_path(self) -> Path:
        return self.workspace_rosdev_path / 'quick_setup_underlay.sh'

    @property
    def docker_entrypoint_sh_quick_setup_underlay_path_env_name(self) -> str:
        return 'ROSDEV_GEN_DOCKER_ENTRYPOINT_SH_QUICK_SETUP_UNDERLAY_PATH'

    @property
    def docker_entrypoint_sh_quick_setup_underlay_path_parent_env_name(self) -> str:
        return 'ROSDEV_GEN_DOCKER_ENTRYPOINT_SH_QUICK_SETUP_UNDERLAY_PATH_PARENT'

    @property
    def docker_entrypoint_sh_setup_overlay_path(self) -> Path:
        return self.workspace_path / 'install' / 'setup.bash'

    @property
    def docker_entrypoint_sh_setup_overlay_path_env_name(self) -> str:
        return 'ROSDEV_GEN_DOCKER_ENTRYPOINT_SH_SETUP_OVERLAY_PATH'

    @property
    def docker_entrypoint_sh_setup_underlay_path(self) -> Path:
        return self.install_symlink_path / 'setup.bash'
    
    @property
    def docker_entrypoint_sh_setup_underlay_path_env_name(self) -> str:
        return 'ROSDEV_GEN_DOCKER_ENTRYPOINT_SH_SETUP_UNDERLAY_PATH'

    @property
    def docker_entrypoint_sh_overlay_path(self) -> Path:
        return self.workspace_rosdev_path / 'docker_entrypoint_sh_overlay.sh'

    @property
    def docker_entrypoint_sh_underlay_path(self) -> Path:
        return self.workspace_rosdev_path / 'docker_entrypoint_sh_underlay.sh'
    
    @property
    def docker_environment_flags(self) -> str:
        return ''.join([
            f'-e {k}={v} '
            for k, v in self.docker_container_environment.items()
        ])

    @property
    def docker_ssh_pam_environment_symlink_container_path(self) -> Path:
        return self.home_path / '.pam_environment'

    @property
    def docker_ssh_pam_environment_path(self) -> Path:
        return self.workspace_rosdev_path / 'pam_environment'

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

    @staticmethod
    def read_text(*, path: Path) -> str:
        return path.read_text()

    @staticmethod
    def read_bytes(*, path: Path) -> bytes:
        return path.read_bytes()

    def write_text(self, *, path: Path, text: str) -> None:
        assert self.stage == 'main', 'Cannot write files outside of main'

        log.debug(f'Writing to {path}, text: \n{text}')
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)

    def write_bytes(self, *, path: Path, text: bytes) -> None:
        assert self.stage == 'main', 'Cannot write files outside of main'

        log.debug(f'Writing to {path}, bytes:\n{text.decode()}')
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(text)
