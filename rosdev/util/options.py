from __future__ import annotations
from atools import memoize
from dataclasses import dataclass
from typing import FrozenSet, Optional


class _TakeFromSelf:
    pass


@memoize
@dataclass(frozen=True)
class Options:
    architecture: str
    architectures: FrozenSet[str]
    asan: bool
    bad_build_num: Optional[int]
    bad_release: str
    build_num: Optional[int]
    clean: bool
    colcon_build_args: Optional[str]
    command: str
    debug: bool
    executable: str
    flavor: str
    gdbserver_port: int
    good_build_num: Optional[int]
    good_release: str
    gui: bool
    interactive: bool
    log_level: str
    name: Optional[str]
    package: str
    ports: FrozenSet[int]
    pull: bool
    release: str
    releases: FrozenSet[str]

    def __call__(
            self,
            architecture: str = _TakeFromSelf,
            architectures: FrozenSet[str] = _TakeFromSelf,
            asan: bool = _TakeFromSelf,
            bad_build_num: Optional[int] = _TakeFromSelf,
            bad_release: str = _TakeFromSelf,
            build_num: Optional[int] = _TakeFromSelf,
            clean: bool = _TakeFromSelf,
            colcon_build_args: Optional[str] = _TakeFromSelf,
            command: str = _TakeFromSelf,
            debug: bool = _TakeFromSelf,
            executable: str = _TakeFromSelf,
            flavor: str = _TakeFromSelf,
            gdbserver_port: int = _TakeFromSelf,
            good_build_num: Optional[int] = _TakeFromSelf,
            good_release: str = _TakeFromSelf,
            gui: bool = _TakeFromSelf,
            interactive: bool = _TakeFromSelf,
            log_level: str = _TakeFromSelf,
            name: Optional[str] = _TakeFromSelf,
            package: str = _TakeFromSelf,
            ports: FrozenSet[int] = _TakeFromSelf,
            pull: bool = _TakeFromSelf,
            release: str = _TakeFromSelf,
            releases: FrozenSet[str] = _TakeFromSelf,
    ) -> Options:

        def __call__inner(**kwargs):
            for k, v in kwargs.items():
                if v is _TakeFromSelf:
                    kwargs[k] = getattr(self, k)

            return Options(**kwargs)

        return __call__inner(
            architecture=architecture,
            architectures=architectures,
            asan=asan,
            bad_build_num=bad_build_num,
            bad_release=bad_release,
            build_num=build_num,
            clean=clean,
            colcon_build_args=colcon_build_args,
            command=command,
            debug=debug,
            executable=executable,
            flavor=flavor,
            gdbserver_port=gdbserver_port,
            good_build_num=good_build_num,
            good_release=good_release,
            gui=gui,
            interactive=interactive,
            log_level=log_level,
            name=name,
            package=package,
            ports=ports,
            pull=pull,
            release=release,
            releases=releases,
        )
