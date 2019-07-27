from dataclasses import asdict, dataclass
from frozendict import frozendict
from pathlib import Path
from typing import Any, Mapping, Optional
from uuid import UUID


@dataclass(frozen=True)
class Options:
    architecture: Optional[str] = None
    build_type: str = 'Debug'

    colcon_build_args: Optional[str] = None
    docker_container_command: str = '/sbin/init'
    docker_container_environment: Mapping[str, str] = frozendict()
    docker_container_name: str = 'rosdev_{architecture}_{ros_build_num}_{base_workspace_hash}'
    # TODO expose ports
    docker_container_ports: Mapping[int, int] = frozendict({22: 22})
    docker_container_volumes: Mapping[str, str] = frozendict()
    docker_image_base_tag: Optional[str] = None
    docker_image_tag: Optional[str] = None
    docker_ssh_workspace_port: Optional[int] = None
    enable_ccache: bool = True
    enable_gui: bool = False
    executable: Optional[str] = None
    flavor: str = 'ros-core'
    ros_container_environment: Mapping[str, str] = frozendict()
    
    idea_base_name: Optional[str] = None
    idea_base_executable_universal_path: Optional[Path] = None

    idea_base_universal_path: Optional[Path] = None
    idea_base_workspace_path: Optional[Path] = None

    idea_c_kdbx_universal_path: Path = Path('{idea_base_universal_path}', 'config', 'c.kdbx')
    idea_c_pwd_universal_path: Path = Path('{idea_base_universal_path}', 'config', 'c.pwd')
    idea_clion_cpp_toolchains_xml_universal_path: Path = Path(
        '{idea_base_universal_path}', 'config', 'options', 'cpp.toolchains.xml'
    )
    idea_deployment_xml_workspace_path: Path = Path('{idea_base_workspace_path}', 'deployment.xml')
    idea_workspace_xml_workspace_path: Path = Path('{idea_base_workspace_path}', 'workspace.xml')
    idea_security_xml_universal_path: Path = Path(
        '{idea_base_universal_path}', 'config', 'options', 'security.xml'
    )
    idea_webservers_xml_universal_path: Path = Path(
        '{idea_base_universal_path}', 'config', 'options', 'webServers.xml'
    )

    idea_uuid: Optional[UUID] = None
    local_setup: Optional[str] = None
    log_level: str = 'INFO'
    package: Optional[str] = None
    pull_docker_image: bool = False
    pull_ros_src: bool = False
    pull_ros_install: bool = False
    ros_build_num: Optional[int] = None
    # TODO make ros_release an enum
    ros_release: str = 'latest'
    replace_docker_container: bool = False
    rosdep_install_args: Optional[str] = None
    sanitizer: Optional[str] = None
    source_ros_overlay_setup_bash: bool = False
    source_ros_underlay_setup_bash: bool = True
    uuid: Optional[str] = None
    base_workspace_hash: Optional[str] = None

    # FIXME allow these to reference eachother out of order
    base_container_path: Optional[Path] = None
    base_workspace_path: Optional[Path] = None
    base_universal_path: Optional[Path] = None

    home_container_path: Optional[Path] = None
    home_universal_path: Optional[Path] = None

    rosdev_container_path: Path = Path('{base_container_path}', '.rosdev')
    rosdev_workspace_path: Path = Path('{base_workspace_path}', '.rosdev')
    rosdev_universal_path: Path = Path('{base_universal_path}', '.rosdev')

    ros_build_num_universal_path: Path = Path(
        '{rosdev_universal_path}', '{architecture}', 'ros_build_num'
    )

    ros_install_container_path: Path = Path(
        '{rosdev_container_path}', 'install'
    )
    ros_install_universal_path: Path = Path(
        '{rosdev_universal_path}', '{architecture}', '{ros_build_num}', 'install'
    )
    ros_install_workspace_path: Path = Path(
        '{rosdev_workspace_path}', 'install'
    )

    ros_src_container_path: Path = Path(
        '{rosdev_container_path}', 'src'
    )
    ros_src_universal_path: Path = Path(
        '{rosdev_universal_path}', '{architecture}', '{ros_build_num}', 'src'
    )
    ros_src_workspace_path: Path = Path(
        '{rosdev_workspace_path}', 'src'
    )

    docker_build_context_workspace_path: Path = Path(
        '{rosdev_workspace_path}', 'docker_build_context'
    )

    docker_bashrc_container_path: Path = Path(
        '{home_container_path}', '.bashrc'
    )
    docker_bashrc_workspace_path: Path = Path(
        '{home_workspace_path}', '.bashrc'
    )

    docker_entrypoint_sh_container_path: Path = Path(
        '/', 'rosdev_docker_entrypoint.sh'
    )
    docker_entrypoint_sh_workspace_path: Path = Path(
        '{rosdev_workspace_path}', 'rosdev_docker_entrypoint.sh'
    )

    ros_overlay_setup_bash_container_path: Path = Path(
        '{base_container_path}', 'install',  'setup.bash'
    )
    ros_overlay_setup_bash_workspace_path: Path = Path(
        '{base_workspace_path}', 'install', 'setup.bash'
    )

    ros_underlay_setup_bash_container_path: Path = Path(
        '{ros_install_container_path}', 'setup.bash'
    )
    ros_underlay_setup_bash_workspace_path: Path = Path(
        '{base_workspace_path}', 'install', 'setup.bash'
    )

    # TODO move these properties to handler as "def get_machine(options)" etc.
    @property
    def machine(self) -> Optional[str]:
        return {
            'amd64': 'x86_64',
            'arm32v7': 'arm',
            'arm64v8': 'aarch64'
        }.get(self.architecture)

    @property
    def operating_system(self) -> Optional[str]:
        return {
            'amd64': 'linux',
            'arm64v8': f'linux-{self.machine}',
        }.get(self.architecture)

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
    
    #def __getattribute__(self, item: str) -> Any:
    #    result = object.__getattribute__(self, item)

    #    if isinstance(result, Path):
    #        result = self.resolve_path(result)
    #
    #    return result
