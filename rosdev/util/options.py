from __future__ import annotations
from atools import memoize
from dataclasses import dataclass
from typing import FrozenSet, Mapping, Optional


class _TakeFromSelf:
    pass


@memoize
@dataclass(frozen=True)
class Options:
    architecture: str
    bad_build_num: Optional[int]
    build_num: Optional[int]
    build_type: Optional[str]
    ccache: bool
    colcon_build_args: Optional[str]
    command: Optional[str]
    executable: str
    flavor: str
    global_setup: Optional[str]
    good_build_num: Optional[int]
    gui: bool
    interactive: bool
    local_setup: Optional[str]
    log_level: str
    name: Optional[str]
    package: Optional[str]
    ports: FrozenSet[int]
    pull: bool
    release: str
    rosdep_install_args: Optional[str]
    sanitizer: Optional[str]
    uuid: Optional[str]
    volumes: Mapping[str, str]

    def __call__(
            self,
            architecture: str = _TakeFromSelf,
            bad_build_num: Optional[int] = _TakeFromSelf,
            build_num: Optional[int] = _TakeFromSelf,
            build_type: Optional[str] = _TakeFromSelf,
            ccache: bool = _TakeFromSelf,
            colcon_build_args: Optional[str] = _TakeFromSelf,
            command: Optional[str] = _TakeFromSelf,
            executable: str = _TakeFromSelf,
            flavor: str = _TakeFromSelf,
            global_setup: Optional[str] = _TakeFromSelf,
            good_build_num: Optional[int] = _TakeFromSelf,
            gui: bool = _TakeFromSelf,
            interactive: bool = _TakeFromSelf,
            local_setup: Optional[str] = _TakeFromSelf,
            log_level: str = _TakeFromSelf,
            name: Optional[str] = _TakeFromSelf,
            package: Optional[str] = _TakeFromSelf,
            ports: FrozenSet[int] = _TakeFromSelf,
            pull: bool = _TakeFromSelf,
            release: str = _TakeFromSelf,
            rosdep_install_args: Optional[str] = _TakeFromSelf,
            sanitizer: Optional[str] = _TakeFromSelf,
            uuid: Optional[str] = _TakeFromSelf,
            volumes: Mapping[str, str] = _TakeFromSelf,
    ) -> Options:

        def __call___inner(**kwargs) -> Options:
            for k, v in kwargs.items():
                if v is _TakeFromSelf:
                    kwargs[k] = getattr(self, k)

            return Options(**kwargs)

        return __call___inner(
            architecture=architecture,
            bad_build_num=bad_build_num,
            build_num=build_num,
            build_type=build_type,
            ccache=ccache,
            colcon_build_args=colcon_build_args,
            command=command,
            executable=executable,
            flavor=flavor,
            global_setup=global_setup,
            good_build_num=good_build_num,
            gui=gui,
            interactive=interactive,
            local_setup=local_setup,
            log_level=log_level,
            name=name,
            package=package,
            ports=ports,
            pull=pull,
            release=release,
            rosdep_install_args=rosdep_install_args,
            sanitizer=sanitizer,
            uuid=uuid,
            volumes=volumes,
        )
