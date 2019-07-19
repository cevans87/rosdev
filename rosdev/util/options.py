from dataclasses import dataclass
from frozendict import frozendict
from typing import FrozenSet, Mapping, Optional


@dataclass(frozen=True)
class Options:
    architecture: str = 'amd64'
    bad_build_num: Optional[int] = None
    build_num: Optional[int] = None
    build_type: str = 'Debug'
    ccache: bool = True
    colcon_build_args: Optional[str] = None
    command: Optional[str] = None
    docker_container_name: Optional[str] = None
    executable: Optional[str] = None
    flavor: str = 'ros-core'
    global_setup: Optional[str] = '.rosdev/install/setup.bash'
    good_build_num: Optional[int] = None
    gui: bool = False
    interactive: bool = False
    local_setup: Optional[str] = None
    log_level: str = 'INFO'
    package: Optional[str] = None
    ports: FrozenSet[int] = frozenset()
    pull_docker_image: bool = False
    pull_src: bool = False
    pull_install: bool = False
    release: str = 'latest'
    replace_docker_container: bool = False
    rosdep_install_args: Optional[str] = None
    sanitizer: Optional[str] = None
    uuid: Optional[str] = None
    volumes: Mapping[str, str] = frozendict()
    
    @memoize
    async def get_build_num(self) -> int:
        build_num = self.build_num
        if (build_num is None) and \
                (not self.pull_install) and \
                (not self.pull_src):
            try:
                paths = sorted(Path(self.global_path).iterdir())
            except FileNotFoundError:
                pass
            else:
                try:
                    build_num = int(str(paths[-1].parts[-1]))
                except (IndexError, ValueError):
                    pass

        if build_num is None:
            build_num = await get_build_num(
                architecture=self.architecture, release=self.release
            )

        return build_num
