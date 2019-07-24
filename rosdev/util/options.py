from dataclasses import asdict, dataclass
from frozendict import frozendict
from pathlib import Path
from typing import FrozenSet, Mapping, Optional


@dataclass(frozen=True)
class Options:
    architecture: Optional[str] = None
    bad_build_num: Optional[int] = None
    build_type: str = 'Debug'
    colcon_build_args: Optional[str] = None
    docker_container_command: Optional[str] = None
    docker_container_environment: Mapping[str, str] = frozendict()
    docker_container_name: str = 'rosdev_{architecture}_{ros_build_num}'
    docker_container_volumes: Mapping[str, str] = frozendict()
    docker_image_base_tag: Optional[str] = None
    docker_image_tag: Optional[str] = None
    enable_ccache: bool = True
    enable_gui: bool = False
    executable: Optional[str] = None
    flavor: str = 'ros-core'
    # TODO remove
    good_build_num: Optional[int] = None
    interactive_docker_container: bool = False
    local_setup: Optional[str] = None
    log_level: str = 'INFO'
    package: Optional[str] = None
    # TODO change name to docker_ports
    ports: FrozenSet[int] = frozenset()
    pull_docker_image: bool = False
    pull_ros_src: bool = False
    pull_ros_install: bool = False
    ros_build_num: Optional[int] = None
    # TODO make ros_release an enum
    ros_release: str = 'latest'
    replace_docker_container: bool = False
    rosdep_install_args: Optional[str] = None
    sanitizer: Optional[str] = None
    source_ros_setup_overlay: bool = False
    source_ros_setup_underlay: bool = True
    uuid: Optional[str] = None

    docker_build_context_path: Optional[Path] = None

    # FIXME allow these to reference eachother out of order
    base_container_path: Optional[Path] = None
    base_workspace_path: Optional[Path] = None
    base_universal_path: Optional[Path] = None

    rosdev_container_path: Path = Path('{base_container_path}', '.rosdev')
    rosdev_workspace_path: Path = Path('{base_workspace_path}', '.rosdev')
    rosdev_universal_path: Path = Path('{base_universal_path}', '.rosdev')

    architecture_container_path: Path = Path('{rosdev_container_path}', '{architecture}')
    architecture_workspace_path: Path = Path('{rosdev_workspace_path}', '{architecture}')
    architecture_universal_path: Path = Path('{rosdev_universal_path}', '{architecture}')

    ros_build_num_container_path: Path = Path('{architecture_container_path}', '{ros_build_num}')
    ros_build_num_universal_path: Path = Path('{architecture_universal_path}', '{ros_build_num}')
    ros_build_num_workspace_path: Path = Path('{architecture_workspace_path}', '{ros_build_num}')

    ros_install_container_path: Path = Path('{ros_build_num_container_path}', 'install')
    ros_install_universal_path: Path = Path('{ros_build_num_universal_path}', 'install')
    ros_install_workspace_path: Path = Path('{ros_build_num_workspace_path}', 'install')

    ros_src_container_path: Path = Path('{ros_build_num_container_path}', 'src')
    ros_src_universal_path: Path = Path('{ros_build_num_universal_path}', 'src')
    ros_src_workspace_path: Path = Path('{ros_build_num_workspace_path}', 'src')

    ros_setup_overlay_container_path: Path = Path('{base_container_path}', 'install', 'setup.bash')
    #ros_setup_overlay_universal_path: Path = Path('{ros_install_universal_path}', 'setup.bash')
    ros_setup_overlay_workspace_path: Path = Path('{base_workspace_path}', 'install', 'setup.bash')

    ros_setup_underlay_container_path: Path = Path('{ros_install_container_path}', 'setup.bash')
    #ros_setup_underlay_universal_path: Path = Path('{ros_install_universal_path}', 'setup.bash')
    ros_setup_underlay_workspace_path: Path = Path('{ros_install_workspace_path}', 'setup.bash')

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

    def _resolve_path(self, path: Path) -> Path:
        return Path(str(path).format(**{
            k: v for k, v in asdict(self).items() if v is not None
        })).expanduser().absolute()

    def resolve_str(self, _str: str) -> str:
        return _str.format(**{k: v for k, v in asdict(self).items() if v is not None})

    def resolve_container_path(self, path: Path) -> Path:
        return self._resolve_path(path)
        
    def resolve_universal_path(self, path: Path) -> Path:
        path = self._resolve_path(path)
        
        assert (
            (path.home() in path.parents) or
            (path.home() == path)
        )

        return path

    def resolve_workspace_path(self, path: Path) -> Path:
        path = self._resolve_path(path)

        assert path.home() in path.parents

        return path
