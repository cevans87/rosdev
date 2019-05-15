from __future__ import annotations
from atools import memoize
from dataclasses import dataclass, field
from frozendict import frozendict
from typing import Dict, FrozenSet, Mapping, Optional


class _TakeFromSelf:
    pass


@memoize
@dataclass(frozen=True)
class Options:
    architecture: str
    asan: bool
    bad_build_num: Optional[int]
    bad_release: str
    build_num: Optional[int]
    ccache: bool
    clean: bool
    colcon_build_args: Optional[str]
    command: Optional[str]
    debug: bool
    executable: str
    flavor: str
    good_build_num: Optional[int]
    good_release: str
    gui: bool
    interactive: bool
    local_setup_path: Optional[str]
    log_level: str
    name: Optional[str]
    package: Optional[str]
    ports: FrozenSet[int]
    pull: bool
    release: str
    uuid: Optional[str]
    volumes: frozendict

    def __call__(
            self,
            architecture: str = _TakeFromSelf,
            asan: bool = _TakeFromSelf,
            bad_build_num: Optional[int] = _TakeFromSelf,
            bad_release: str = _TakeFromSelf,
            build_num: Optional[int] = _TakeFromSelf,
            ccache: bool = _TakeFromSelf,
            clean: bool = _TakeFromSelf,
            colcon_build_args: Optional[str] = _TakeFromSelf,
            command: Optional[str] = _TakeFromSelf,
            debug: bool = _TakeFromSelf,
            executable: str = _TakeFromSelf,
            flavor: str = _TakeFromSelf,
            good_build_num: Optional[int] = _TakeFromSelf,
            good_release: str = _TakeFromSelf,
            gui: bool = _TakeFromSelf,
            interactive: bool = _TakeFromSelf,
            local_setup_path: Optional[str] = _TakeFromSelf,
            log_level: str = _TakeFromSelf,
            name: Optional[str] = _TakeFromSelf,
            package: Optional[str] = _TakeFromSelf,
            ports: FrozenSet[int] = _TakeFromSelf,
            pull: bool = _TakeFromSelf,
            release: str = _TakeFromSelf,
            uuid: Optional[str] = _TakeFromSelf,
            volumes: frozendict = _TakeFromSelf,
    ) -> Options:

        def __call___inner(**kwargs) -> Options:
            for k, v in kwargs.items():
                if v is _TakeFromSelf:
                    kwargs[k] = getattr(self, k)

            return Options(**kwargs)

        return __call___inner(
            architecture=architecture,
            asan=asan,
            bad_build_num=bad_build_num,
            bad_release=bad_release,
            build_num=build_num,
            ccache=ccache,
            clean=clean,
            colcon_build_args=colcon_build_args,
            command=command,
            debug=debug,
            executable=executable,
            flavor=flavor,
            good_build_num=good_build_num,
            good_release=good_release,
            gui=gui,
            interactive=interactive,
            local_setup_path=local_setup_path,
            log_level=log_level,
            name=name,
            package=package,
            ports=ports,
            pull=pull,
            release=release,
            uuid=uuid,
            volumes=volumes,
        )
