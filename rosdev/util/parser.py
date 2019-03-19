from __future__ import annotations
from argcomplete import autocomplete
from argparse import ArgumentParser, SUPPRESS
import ast
from collections import ChainMap
from dataclasses import dataclass, field
from importlib import import_module
import logging
import os
from pathlib import Path
from typing import Awaitable, FrozenSet, List, Optional


@dataclass(frozen=True)
class Defaults:
    architecture: str = 'amd64'
    architectures: FrozenSet[str] = frozenset({architecture})
    asan: bool = False
    bad_build_num: Optional[int] = None
    bad_release: str = 'latest'
    build_num: Optional[int] = None
    clean: bool = False
    colcon_build_args: Optional[str] = None
    command: str = ''
    debug: bool = False
    executable: str = ''
    fast: bool = False
    flavor: str = 'ros-core'
    gdbserver_port: int = 1337
    good_build_num: Optional[int] = None
    good_release: str = 'crystal'
    gui: bool = False
    interactive: bool = False
    log_level: str = 'INFO'
    package: str = ''
    ports: FrozenSet[int] = frozenset()
    release: str = 'latest'
    releases: FrozenSet[str] = frozenset({release})

    @staticmethod
    def from_workspace() -> Defaults:
        # noinspection PyBroadException
        try:
            with open(f'{Path.home()}/.rosdev/workspace', 'r') as workspace_f_in:
                global_overrides = ast.literal_eval(workspace_f_in.read())
        except Exception:
            global_overrides = {}
        # noinspection PyBroadException
        try:
            with open(f'{os.getcwd()}/.rosdev/workspace', 'r') as workspace_f_in:
                local_overrides = ast.literal_eval(workspace_f_in.read())
        except Exception:
            local_overrides = {}

        return Defaults(**ChainMap(local_overrides, global_overrides))


defaults = Defaults.from_workspace()


def gen_flag_parser() -> ArgumentParser:
    return ArgumentParser(add_help=False)


@dataclass(frozen=True)
class Positional:
    command: ArgumentParser = field(default_factory=gen_flag_parser)
    executable: ArgumentParser = field(default_factory=gen_flag_parser)
    package: ArgumentParser = field(default_factory=gen_flag_parser)


positional = Positional()
positional.command.add_argument('command')
positional.executable.add_argument('executable')
positional.package.add_argument('package')


@dataclass(frozen=True)
class Flag:
    architecture: ArgumentParser = field(default_factory=gen_flag_parser)
    architectures: ArgumentParser = field(default_factory=gen_flag_parser)
    asan: ArgumentParser = field(default_factory=gen_flag_parser)
    bad_build_num: ArgumentParser = field(default_factory=gen_flag_parser)
    bad_release: ArgumentParser = field(default_factory=gen_flag_parser)
    build_num: ArgumentParser = field(default_factory=gen_flag_parser)
    clean: ArgumentParser = field(default_factory=gen_flag_parser)
    colcon_build_args: ArgumentParser = field(default_factory=gen_flag_parser)
    debug: ArgumentParser = field(default_factory=gen_flag_parser)
    fast: ArgumentParser = field(default_factory=gen_flag_parser)
    flavor: ArgumentParser = field(default_factory=gen_flag_parser)
    gdbserver_port: ArgumentParser = field(default_factory=gen_flag_parser)
    good_build_num: ArgumentParser = field(default_factory=gen_flag_parser)
    good_release: ArgumentParser = field(default_factory=gen_flag_parser)
    gui: ArgumentParser = field(default_factory=gen_flag_parser)
    interactive: ArgumentParser = field(default_factory=gen_flag_parser)
    log_level: ArgumentParser = field(default_factory=gen_flag_parser)
    port: ArgumentParser = field(default_factory=gen_flag_parser)
    ports: ArgumentParser = field(default_factory=gen_flag_parser)
    release: ArgumentParser = field(default_factory=gen_flag_parser)
    releases: ArgumentParser = field(default_factory=gen_flag_parser)


flag = Flag()


@dataclass(frozen=True)
class Choices:
    architecture = sorted({'amd64', 'arm32v7', 'arm64v8'})
    bad_release = sorted({'bionic', 'crystal'}) + ['latest']
    flavor = sorted({'desktop', 'desktop-full', 'robot', 'perception', 'ros-core', 'ros-base'})
    good_release = sorted({'ardent', 'bionic', 'crystal'})
    # noinspection PyProtectedMember
    log_level = [name for _, name in sorted(logging._levelToName.items())]
    release = sorted({'ardent', 'bionic', 'crystal', 'kinetic', 'melodic'}) + ['latest']


choices = Choices()

flag.architecture.add_argument(
    '--architecture', '-a',
    default=defaults.architecture,
    choices=choices.architecture,
    help=f'Architecture to build. Currently: {defaults.architecture}',
)
flag.architectures.add_argument(
    '--architectures',
    default=defaults.architectures,
    nargs='+',
    choices=choices.architecture,
    help=f'List of architectures to build. Currently: {defaults.architectures}',
)

asan_group = flag.asan.add_mutually_exclusive_group()
asan_group.add_argument(
    '--asan',
    action='store_true',
    help=(
        SUPPRESS if defaults.asan else
        f'Build with Address Sanitizer enabled. Currently: {defaults.asan}'
    )
)
asan_group.add_argument(
    '--no-asan',
    action='store_false',
    dest='asan',
    help=(
        SUPPRESS if not defaults.asan else
        f'Build with Address Sanitizer disabled, Currently: {not defaults.asan}'
    )
)

flag.bad_release.add_argument(
    '--bad-release',
    default=defaults.bad_release,
    choices=choices.bad_release,
    help=f'Bad release to compare. Currently: {defaults.bad_release}'
)

flag.bad_build_num.add_argument(
    '--bad-build-num',
    type=int,
    default=defaults.bad_build_num,
    help=f'Bad build number to compare. Supersedes --bad-build'
)

flag.build_num.add_argument(
    '--build-num',
    type=int,
    default=defaults.build_num,
    help=(
        f'Use specified build from OSRF build farm instead of {defaults.release}. '
        f'Currently: {defaults.build_num}'
    ),
)

clean_group = flag.clean.add_mutually_exclusive_group()
clean_group.add_argument(
    '--clean',
    action='store_true',
    default=defaults.clean,
    help=(
        SUPPRESS if defaults.clean else
        f'Start bash environment without sourcing a ROS setup.bash. Currently: {defaults.clean}'
    )
)
clean_group.add_argument(
    '--no-clean',
    action='store_false',
    dest='clean',
    default=not defaults.clean,
    help=(
        SUPPRESS if not defaults.clean else
        f'Start bash environment with sourcing a ROS setup.bash. Currently: {not defaults.clean}'
    )
)

flag.colcon_build_args.add_argument(
    '--colcon-build-args',
    default=defaults.colcon_build_args,
    help=f'Additional args to pass to colcon build'
)

debug_group = flag.debug.add_mutually_exclusive_group()
debug_group.add_argument(
    '--debug',
    action='store_true',
    default=defaults.debug,
    help=(
        SUPPRESS if defaults.debug else
        f'Build with debug enabled. Currrently: {defaults.debug}'
    )
)
debug_group.add_argument(
    '--no-debug',
    action='store_false',
    dest='debug',
    default=not defaults.debug,
    help=(
        SUPPRESS if not defaults.debug else
        f'Build with debug disabled. Currently: {defaults.debug}'
    )
)

fast_group = flag.fast.add_mutually_exclusive_group()
fast_group.add_argument(
    '--fast', '-f',
    action='store_true',
    default=defaults.fast,
    help=(
        SUPPRESS if defaults.fast else
        f'Build from local docker images if able. Currently: {defaults.fast}'
    )
)
fast_group.add_argument(
    '--no-fast',
    action='store_false',
    dest='fast',
    default=not defaults.fast,
    help=(
        SUPPRESS if not defaults.fast else
        f'Do not build from local docker images if able. Currently: {defaults.fast}'
    )
)

flag.flavor.add_argument(
    '--flavor',
    default=defaults.flavor,
    choices=choices.flavor,
    help=f'Linux flavor. Currently: {defaults.flavor}'
)

flag.gdbserver_port.add_argument(
    '--gdbserver-port',
    type=int,
    default=defaults.gdbserver_port,
    help=f'Currently: {defaults.gdbserver_port}'
)
flag.good_build_num.add_argument(
    '--good-build-num',
    type=int,
    default=defaults.good_build_num,
    help=f'Good build number to compare. Supersedes --good-build'
)

flag.good_release.add_argument(
    '--good-release',
    default=defaults.good_release,
    choices=choices.good_release,
    help=f'Good release to compare. Currently: {defaults.good_release}'
)

gui_group = flag.gui.add_mutually_exclusive_group()
gui_group.add_argument(
    '--gui',
    default=defaults.gui,
    action='store_true',
    help=(
        SUPPRESS if defaults.gui else
        f'Allow container to use host X11 server. Currently: {defaults.gui}'
    )
)
gui_group.add_argument(
    '--no-gui',
    dest='gui',
    default=not defaults.gui,
    action='store_false',
    help=(
        SUPPRESS if not defaults.gui else
        f'Do not allow container to use host X11 server. Currently: {defaults.gui}'
    )
)

interactive_group = flag.interactive.add_mutually_exclusive_group()
interactive_group.add_argument(
    '--interactive',
    action='store_true',
    help=(
        SUPPRESS if defaults.interactive else
        f'Make derived docker container interactive. Currently: {defaults.interactive}'
    )
)
interactive_group.add_argument(
    '--no-interactive',
    dest='interactive',
    action='store_false',
    default=not defaults.interactive,
    help=(
        SUPPRESS if not defaults.interactive else
        f'Do not make derived docker container interactive. Currently: {defaults.interactive}'
    )
)

flag.log_level.add_argument(
    '--log-level',
    default=defaults.log_level,
    choices=choices.log_level,
    help=f'Currently: {defaults.log_level}',
)

flag.release.add_argument(
    '--release', '-r',
    default=defaults.release,
    choices=choices.release,
    help=f'ROS release to build. Currently: {defaults.release}',
)

flag.releases.add_argument(
    '--releases',
    default=defaults.releases,
    nargs='+',
    choices=choices.release,
    help=f'List of ROS releases to build. Currently: {defaults.releases}',
)

flag.ports.add_argument(
    '--ports', '-p',
    nargs='*',
    type=int,
    default=sorted(defaults.ports),
    help=f'List of ports to expose in docker container. Currently: {defaults.ports}'
)

rosdev_parser = ArgumentParser(parents=[flag.log_level])
rosdev_subparsers = rosdev_parser.add_subparsers(required=True)
rosdev_bash_parser = rosdev_subparsers.add_parser(
    'bash', parents=[
        flag.architecture,
        flag.build_num,
        flag.clean,
        flag.fast,
        flag.flavor,
        flag.gui,
        flag.log_level,
        flag.ports,
        flag.release,
    ]
)
rosdev_bash_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.bash').Bash)
rosdev_bisect_parser = rosdev_subparsers.add_parser(
    'bisect',
    parents=[
        positional.command,
        flag.architecture,
        flag.asan,
        flag.bad_release,
        flag.bad_build_num,
        flag.colcon_build_args,
        flag.debug,
        flag.fast,
        flag.good_build_num,
        flag.good_release,
        flag.release
    ]
)
rosdev_bisect_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.bisect').Bisect)
rosdev_clion_parser = rosdev_subparsers.add_parser(
    'clion',
    parents=[
        flag.architecture,
        flag.build_num,
        flag.release
    ]
)
rosdev_clion_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.clion').Clion)

rosdev_gazebo_parser = rosdev_subparsers.add_parser(
    'gazebo',
    parents=[
        flag.fast,
        flag.log_level,
        flag.release,
    ]
)
rosdev_gazebo_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gazebo').Gazebo)

rosdev_gdbserver_parser = rosdev_subparsers.add_parser(
    'gdbserver', parents=[
        positional.package,
        positional.executable,
        flag.architecture,
        flag.build_num,
        flag.fast,
        flag.gdbserver_port,
        flag.release,
    ]
)
rosdev_gdbserver_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gdbserver').Gdbserver)
rosdev_gen_parser = rosdev_subparsers.add_parser(
    'gen', parents=[])
rosdev_gen_subparsers = rosdev_gen_parser.add_subparsers(required=True)
rosdev_gen_colcon_parser = rosdev_gen_subparsers.add_parser(
    'colcon', parents=[])
rosdev_gen_colcon_subparsers = rosdev_gen_colcon_parser.add_subparsers(required=True)
rosdev_gen_colcon_build_parser = rosdev_gen_colcon_subparsers.add_parser(
    'build', parents=[
        flag.architecture,
        flag.asan,
        flag.build_num,
        flag.clean,
        flag.colcon_build_args,
        flag.debug,
        flag.fast,
        flag.release
    ])
rosdev_gen_colcon_build_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gen.colcon.build').Build)
rosdev_gen_docker_parser = rosdev_gen_subparsers.add_parser(
    'docker', parents=[])
rosdev_gen_docker_subparsers = rosdev_gen_docker_parser.add_subparsers(required=True)
rosdev_gen_docker_container_parser = rosdev_gen_docker_subparsers.add_parser(
    'container',
    parents=[
        positional.command,
        flag.architecture,
        flag.clean,
        flag.fast,
        flag.interactive,
        flag.ports,
        flag.release
    ]
)
rosdev_gen_docker_container_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gen.docker.container').Container)
rosdev_gen_global_parser = rosdev_gen_subparsers.add_parser(
    'global', parents=[]
)
rosdev_gen_global_subparsers = rosdev_gen_global_parser.add_subparsers(required=True)
rosdev_gen_global_workspace_parser = rosdev_gen_global_subparsers.add_parser(
    'workspace',
    parents=flag.__dict__.values()
)
rosdev_gen_global_workspace_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gen.global.workspace').Workspace)
rosdev_gen_docker_images_parser = rosdev_gen_docker_subparsers.add_parser(
    'images', parents=[flag.architectures, flag.fast, flag.log_level, flag.releases])
rosdev_gen_docker_images_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gen.docker.images').Images)
rosdev_gen_install_parser = rosdev_gen_subparsers.add_parser(
    'install', parents=[flag.architecture, flag.build_num, flag.fast, flag.log_level, flag.release])
rosdev_gen_install_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gen.install').Install)
rosdev_gen_local_parser = rosdev_gen_subparsers.add_parser(
    'local', parents=[]
)
rosdev_gen_local_subparsers = rosdev_gen_local_parser.add_subparsers(required=True)
rosdev_gen_local_workspace_parser = rosdev_gen_local_subparsers.add_parser(
    'workspace',
    parents=flag.__dict__.values()
)
rosdev_gen_local_workspace_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gen.global.workspace').Workspace)


def get_handler(args: Optional[List[str]]) -> Awaitable:
    autocomplete(rosdev_parser)

    import sys
    args = args if args is not None else sys.argv[1:]
    try:
        args = rosdev_parser.parse_args(args)
    except Exception:
        rosdev_parser.print_usage()
        raise

    for k, v in args.__dict__.items():
        if isinstance(v, list):
            setattr(args, k, frozenset(v))

    handler = args.get_handler()
    del args.__dict__['get_handler']

    from collections import ChainMap
    from rosdev.util.options import Options
    return handler(Options(**ChainMap(args.__dict__, defaults.__dict__)))
