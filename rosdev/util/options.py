from __future__ import annotations
from dataclasses import dataclass, replace
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
    # TODO change this to pull_docker_image
    pull_docker_image: bool = False
    pull_src: bool = False
    pull_install: bool = False
    release: str = 'latest'
    # TODO change this to replace_docker_container
    replace_docker_container: bool = False
    rosdep_install_args: Optional[str] = None
    sanitizer: Optional[str] = None
    uuid: Optional[str] = None
    volumes: Mapping[str, str] = frozendict()

    def __call__(self, **kwargs) -> Options:
        return replace(self, **kwargs)
