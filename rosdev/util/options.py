from dataclasses import asdict, dataclass
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
    build_num: Optional[int] = None
    build_type: str = 'Debug'

    docker_entrypoint_sh_quick_overlay: bool = True
    docker_entrypoint_sh_quick_underlay: bool = True
    docker_entrypoint_sh_setup_overlay: bool = False
    docker_entrypoint_sh_setup_underlay: bool = True

    colcon_build_args: Optional[str] = None
    container_path: Optional[Path] = None
    docker_container_command: str = '/sbin/init'
    docker_container_environment: Mapping[str, str] = frozendict()
    docker_container_ports: Mapping[int, int] = frozendict()  # container:host
    docker_container_volumes: Mapping[str, str] = frozendict()  # host:container
    docker_image_base_tag: Optional[str] = None
    docker_ssh_port: Optional[int] = None

    dry_run: bool = False
    # TODO rename to enable_docker_container_ccache
    enable_ccache: bool = True
    # TODO rename to enable_docker_container_gui
    enable_gui: bool = False
    executable: Optional[str] = None
    # TODO find a better name for this
    flavor: str = 'ros-core'

    idea_ide_name: Optional[str] = None
    idea_ide_start_universal_path: Optional[Path] = None
    idea_universal_path: Optional[Path] = None

    idea_uuid: Optional[UUID] = None
    log_level: str = 'INFO'
    package: Optional[str] = None
    pull_build: bool = False
    pull_docker_image: bool = False
    release: Optional[str] = None
    replace_docker_container: bool = False
    rosdep_install_args: Optional[str] = None
    run_main: bool = True
    run_validate_options: bool = True
    sanitizer: Optional[str] = None

    reuse_environment: bool = True
    reuse_docker_ssh_pam_environment: bool = True

    source_ros_overlay_setup_bash: bool = False
    source_ros_underlay_setup_bash: bool = True

    stage: Optional[str] = None
    universal_path: Optional[Path] = None
    workspace_hash: Optional[str] = None

    workspace_path: Optional[Path] = None

    home_container_path: Optional[Path] = None
    home_universal_path: Optional[Path] = None

    @property
    def docker_image_tag(self) -> str:
        return f'rosdev:{self.architecture}_{self.release}'

    @property
    def docker_container_name(self) -> str:
        return f'rosdev_{self.workspace_hash}_{self.architecture}_{self.release}_{self.build_num}'

    @property
    def build_num_universal_path(self) -> Path:
        return self.resolve_path(
            Path(
                self.rosdev_universal_path, 
                f'{self.architecture}',
                f'{self.release}',
                'build_num'
            )
        )
    
    @property
    def docker_ssh_port_workspace_path(self) -> Path:
        return self.resolve_path(Path(self.rosdev_workspace_path, 'docker_ssh_port'))

    @property
    def idea_workspace_path(self) -> Path:
        return self.resolve_path(Path(self.workspace_path, '.idea'))

    @property
    def idea_c_kdbx_universal_path(self) -> Path:
        return self.resolve_path(Path(self.idea_universal_path, 'c.kdbx'))
    
    @property
    def idea_c_pwd_universal_path(self) -> Path:
        return self.resolve_path(Path(self.idea_universal_path, 'c.pwd'))
    
    @property
    def idea_clion_cpp_toolchains_xml_universal_path(self) -> Path: 
        return self.resolve_path(Path(self.idea_universal_path, 'options', 'cpp.toolchains.xml'))

    @property
    def idea_clion_webservers_xml_universal_path(self) -> Path:
        return self.resolve_path(Path(self.idea_universal_path, 'options', 'webServers.xml'))

    @property
    def idea_pycharm_jdk_table_xml_sftp_uri(self) -> str:
        return f'sftp://{os.getlogin()}@localhost:{self.docker_ssh_port}/usr/bin/python'

    @property
    def idea_pycharm_jdk_table_xml_ssh_uri(self) -> str:
        return f'ssh://{os.getlogin()}@localhost:{self.docker_ssh_port}/usr/bin/python'
    
    @property
    def idea_pycharm_misc_xml_workspace_path(self) -> Path:
        return self.resolve_path(Path(self.idea_workspace_path, 'misc.xml'))

    @property
    def idea_pycharm_webservers_xml_workspace_path(self) -> Path:
        return self.resolve_path(Path(self.idea_workspace_path, 'webServers.xml'))

    @property
    def idea_clion_deployment_xml_workspace_path(self) -> Path:
        return self.resolve_path(Path(self.idea_workspace_path, 'deployment.xml'))

    @property
    def idea_pycharm_deployment_xml_workspace_path(self) -> Path:
        return self.resolve_path(Path(self.idea_workspace_path, 'deployment.xml'))

    @property
    def idea_pycharm_jdk_table_xml_universal_path(self) -> Path:
        return self.resolve_path(Path(self.idea_universal_path, 'options', 'jdk.table.xml'))

    @property
    def idea_workspace_xml_workspace_path(self) -> Path:
        return self.resolve_path(Path(self.idea_workspace_path, 'workspace.xml'))
    
    @property
    def idea_security_xml_universal_path(self) -> Path:
        return self.resolve_path(Path(self.idea_universal_path, 'options', 'security.xml'))

    @property
    def idea_clion_webservers_name(self) -> str:
        return f'rosdev_{self.idea_ide_name}_{self.workspace_hash} ({self.idea_uuid})'

    @property
    def idea_pycharm_webservers_name(self) -> str:
        return f'rosdev_{self.idea_ide_name}_{self.workspace_hash}'
    
    @property
    def idea_uuid_workspace_path(self) -> Path:
        return self.resolve_path(Path(self.rosdev_workspace_path, 'idea_uuid'))

    @property
    def idea_webservers_xml_workspace_path(self) -> Path:
        return self.resolve_path(Path(self.idea_workspace_path, 'webServers.xml'))

    @property
    def docker_gdbinit_container_path(self) -> Path:
        return self.resolve_path(Path(self.home_container_path, '.gdbinit'))

    @property
    def docker_gdbinit_workspace_path(self) -> Path:
        return self.resolve_path(Path(self.rosdev_workspace_path, 'gdbinit'))

    @property
    def rosdev_container_path(self) -> Path:
        return self.resolve_path(Path(self.container_path, '.rosdev'))

    @property
    def rosdev_workspace_path(self) -> Path:
        return self.resolve_path(Path(self.workspace_path, '.rosdev'))
    
    @property
    def rosdev_universal_path(self) -> Path:
        return self.resolve_path(Path(self.universal_path, '.rosdev'))

    @property
    def install_container_path(self) -> Path:
        return self.resolve_path(Path(self.rosdev_container_path, 'install'))

    @property
    def install_universal_path(self) -> Path:
        return self.resolve_path(
            Path(
                self.rosdev_universal_path,
                f'{self.architecture}',
                f'{self.release}',
                f'{self.build_num}',
                'install'
            )
        )

    @property
    def install_workspace_path(self) -> Path: 
        return self.resolve_path(Path(self.rosdev_workspace_path, 'install'))

    @property
    def src_container_path(self) -> Path:
        return self.resolve_path(Path(self.rosdev_container_path, 'src'))
    
    @property
    def src_universal_path(self) -> Path: 
        return self.resolve_path(
            Path(
                self.rosdev_universal_path, 
                f'{self.architecture}',
                f'{self.release}',
                f'{self.build_num}',
                'src'
            )
        )

    @property
    def src_workspace_path(self) -> Path: 
        return self.resolve_path(Path(self.rosdev_workspace_path, 'src'))
    
    @property
    def overrides_path(self) -> Path:
        return self.resolve_path(Path(self.rosdev_workspace_path, 'overrides'))

    @property
    def docker_bashrc_container_path(self) -> Path: 
        return self.resolve_path(Path(self.rosdev_container_path, 'bashrc'))
    
    @property
    def docker_bashrc_symlinked_path(self) -> Path: 
        return self.resolve_path(Path(self.home_container_path, '.bashrc'))
    
    @property
    def docker_bashrc_workspace_path(self) -> Path:
        return self.resolve_path(Path(self.rosdev_workspace_path, 'bashrc'))

    @property
    def docker_dockerfile_workspace_path(self) -> Path:
        return self.resolve_path(Path(self.rosdev_workspace_path, 'Dockerfile'))

    @property
    def docker_entrypoint_sh_container_path(self) -> Path:
        return self.resolve_path(Path(self.rosdev_container_path, 'rosdev_docker_entrypoint.sh'))
    
    @property
    def docker_entrypoint_sh_workspace_path(self) -> Path: 
        return self.resolve_path(Path(self.rosdev_workspace_path, 'rosdev_docker_entrypoint.sh'))

    @property
    def docker_entrypoint_sh_overlay_sh_container_path(self) -> Path:
        return self.resolve_path(
            Path(self.rosdev_container_path, 'docker_entrypoint_sh_overlay.sh')
        )

    @property
    def docker_entrypoint_sh_underlay_sh_container_path(self) -> Path:
        return self.resolve_path(
            Path(self.rosdev_container_path, 'docker_entrypoint_sh_underlay.sh')
        )

    @property
    def docker_entrypoint_sh_setup_overlay_container_path(self) -> Path:
        return self.resolve_path(Path(f'{self.container_path}', 'install',  'setup.bash'))

    @property
    def docker_entrypoint_sh_setup_overlay_workspace_path(self) -> Path:
        return self.resolve_path(Path(f'{self.workspace_path}', 'install', 'setup.bash'))

    @property
    def docker_entrypoint_sh_setup_underlay_container_path(self) -> Path:
        return self.resolve_path(Path(f'{self.install_container_path}', 'setup.bash'))

    @property
    def docker_entrypoint_sh_setup_underlay_workspace_path(self) -> Path:
        return self.resolve_path(Path('{workspace_path}', 'install', 'setup.bash'))

    @property
    def docker_ssh_pam_environment_container_path(self) -> Path:
        return self.resolve_path(Path(self.rosdev_container_path, 'pam_environment'))

    @property
    def docker_ssh_pam_environment_workspace_path(self) -> Path:
        return self.resolve_path(Path(self.rosdev_workspace_path, 'pam_environment'))

    @property
    def machine(self) -> str:
        return {
            'amd64': 'x86_64',
            'arm32v7': 'arm',
            'arm64v8': 'aarch64'
        }[self.architecture]

    @property
    def operating_system(self) -> str:
        return {
            'amd64': 'linux',
            'arm32v7': f'linux-armhf',
            'arm64v8': f'linux-aarch64',
        }[self.architecture]

    def resolve_str(self, _str: str) -> str:
        # TODO py38 walrus loop
        old_str = ''
        while old_str != _str:
            old_str = _str
            _str = _str.format(**{k: v for k, v in asdict(self).items() if v is not None})

        return _str

    def resolve_path(self, path: Path) -> Path:
        # TODO py38 walrus loop
        old_path = Path()
        while old_path != path:
            old_path = path
            path = Path(str(path).format(**{
                k: v for k, v in asdict(self).items() if v is not None
            })).expanduser().absolute()

        return path
    
    @staticmethod
    def read_text(*, path: Path) -> str:
        return path.read_text()

    @staticmethod
    def read_bytes(*, path: Path) -> bytes:
        return path.read_bytes()

    def write_text(self, *, path: Path, text: str) -> None:
        assert self.stage == 'main', 'Cannot write files outside of main'

        if not self.dry_run:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text)

    def write_bytes(self, *, path: Path, text: bytes) -> None:
        assert self.stage == 'main', 'Cannot write files outside of main'

        if not self.dry_run:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(text)
