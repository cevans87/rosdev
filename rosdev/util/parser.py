from __future__ import annotations
from argcomplete import autocomplete
from argparse import Action, ArgumentParser, Namespace, SUPPRESS
import ast
from collections import ChainMap
from dataclasses import asdict, dataclass, field
from frozendict import frozendict
from importlib import import_module
import logging
import os
from pathlib import Path
from typing import Awaitable, Dict, FrozenSet, List, Optional


@dataclass(frozen=True)
class Defaults:
    architecture: str = 'amd64'
    asan: bool = False
    bad_build_num: Optional[int] = None
    bad_release: str = 'latest'
    build_num: Optional[int] = None
    clean: bool = False
    colcon_build_args: Optional[str] = None
    command: Optional[str] = None
    debug: bool = False
    executable: str = ''
    flavor: str = 'ros-core'
    good_build_num: Optional[int] = None
    good_release: str = 'crystal'
    gui: bool = False
    interactive: bool = False
    log_level: str = 'INFO'
    name: Optional[str] = None
    package: str = ''
    ports: FrozenSet[int] = frozenset()
    pull: bool = False
    release: str = 'latest'
    volumes: frozendict = frozendict()

    @staticmethod
    def _get_local_overrides() -> Dict:
        # noinspection PyBroadException
        try:
            with open(f'{os.getcwd()}/.rosdev/defaults', 'r') as overrides_f_in:
                return ast.literal_eval(overrides_f_in.read())
        except Exception:
            return {}

    @staticmethod
    def _get_global_overrides() -> Dict:
        # noinspection PyBroadException
        try:
            with open(f'{Path.home()}/.rosdev/defaults', 'r') as overrides_f_in:
                return ast.literal_eval(overrides_f_in.read())
        except Exception:
            return {}

    @staticmethod
    def with_local_overrides() -> Defaults:
        return Defaults(**Defaults._get_local_overrides())

    @staticmethod
    def with_global_overrides() -> Defaults:
        return Defaults(**Defaults._get_global_overrides())

    @staticmethod
    def with_overrides() -> Defaults:
        return Defaults(
            **ChainMap(
                Defaults._get_local_overrides(),
                Defaults._get_global_overrides()
            )
        )


defaults = Defaults.with_overrides()


def gen_flag_parser() -> ArgumentParser:
    return ArgumentParser(add_help=False)


@dataclass(frozen=True)
class Positional:
    command: ArgumentParser = field(default_factory=gen_flag_parser)
    executable: ArgumentParser = field(default_factory=gen_flag_parser)
    package: ArgumentParser = field(default_factory=gen_flag_parser)

    def __post_init__(self) -> None:
        self.command.add_argument('command')
        self.executable.add_argument('executable')
        self.package.add_argument('package')


positional = Positional()


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


@dataclass(frozen=True)
class Flag:
    architecture: ArgumentParser = field(default_factory=gen_flag_parser)
    asan: ArgumentParser = field(default_factory=gen_flag_parser)
    bad_build_num: ArgumentParser = field(default_factory=gen_flag_parser)
    bad_release: ArgumentParser = field(default_factory=gen_flag_parser)
    build_num: ArgumentParser = field(default_factory=gen_flag_parser)
    clean: ArgumentParser = field(default_factory=gen_flag_parser)
    colcon_build_args: ArgumentParser = field(default_factory=gen_flag_parser)
    debug: ArgumentParser = field(default_factory=gen_flag_parser)
    flavor: ArgumentParser = field(default_factory=gen_flag_parser)
    good_build_num: ArgumentParser = field(default_factory=gen_flag_parser)
    good_release: ArgumentParser = field(default_factory=gen_flag_parser)
    gui: ArgumentParser = field(default_factory=gen_flag_parser)
    interactive: ArgumentParser = field(default_factory=gen_flag_parser)
    log_level: ArgumentParser = field(default_factory=gen_flag_parser)
    name: ArgumentParser = field(default_factory=gen_flag_parser)
    port: ArgumentParser = field(default_factory=gen_flag_parser)
    ports: ArgumentParser = field(default_factory=gen_flag_parser)
    pull: ArgumentParser = field(default_factory=gen_flag_parser)
    release: ArgumentParser = field(default_factory=gen_flag_parser)
    volumes: ArgumentParser = field(default_factory=gen_flag_parser)

    def __post_init__(self) -> None:
        self.architecture.add_argument(
            '--architecture', '-a',
            default=defaults.architecture,
            choices=choices.architecture,
            help=f'Architecture to build. Currently: {defaults.architecture}',
        )

        asan_group = self.asan.add_mutually_exclusive_group()
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

        self.bad_release.add_argument(
            '--bad-release',
            default=defaults.bad_release,
            choices=choices.bad_release,
            help=f'Bad release to compare. Currently: {defaults.bad_release}'
        )

        self.bad_build_num.add_argument(
            '--bad-build-num',
            type=int,
            default=defaults.bad_build_num,
            help=f'Bad build number to compare. Supersedes --bad-build'
        )

        self.build_num.add_argument(
            '--build-num',
            type=int,
            default=defaults.build_num,
            help=(
                f'Use specified build from OSRF build farm instead of {defaults.release}. '
                f'Currently: {defaults.build_num}'
            ),
        )

        clean_group = self.clean.add_mutually_exclusive_group()
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

        self.colcon_build_args.add_argument(
            '--colcon-build-args',
            default=defaults.colcon_build_args,
            help=f'Additional args to pass to colcon build'
        )

        debug_group = self.debug.add_mutually_exclusive_group()
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

        self.flavor.add_argument(
            '--flavor',
            default=defaults.flavor,
            choices=choices.flavor,
            help=f'Linux flavor. Currently: {defaults.flavor}'
        )

        self.good_build_num.add_argument(
            '--good-build-num',
            type=int,
            default=defaults.good_build_num,
            help=f'Good build number to compare. Supersedes --good-build'
        )

        self.good_release.add_argument(
            '--good-release',
            default=defaults.good_release,
            choices=choices.good_release,
            help=f'Good release to compare. Currently: {defaults.good_release}'
        )

        gui_group = self.gui.add_mutually_exclusive_group()
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

        interactive_group = self.interactive.add_mutually_exclusive_group()
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

        self.log_level.add_argument(
            '--log-level',
            default=defaults.log_level,
            choices=choices.log_level,
            help=f'Currently: {defaults.log_level}',
        )

        self.name.add_argument(
            '--name',
            default=defaults.name,
            help=(
                f'Name to assign to docker container. Existing container with name will be removed. '
                f'Currently: {defaults.name}'
            ),
        )

        # TODO add PortsAction to allow <host>:<container> mapping. See VolumesAction for reference.
        self.ports.add_argument(
            '--ports', '-p',
            nargs='*',
            type=int,
            default=sorted(defaults.ports),
            help=f'List of ports to expose in docker container. Currently: {defaults.ports}'
        )
        pull_group = self.pull.add_mutually_exclusive_group()
        pull_group.add_argument(
            '--pull',
            action='store_true',
            default=defaults.pull,
            help=(
                SUPPRESS if defaults.pull else
                f'Pull newer docker image. Currently: {defaults.pull}'
            )
        )
        pull_group.add_argument(
            '--no-pull',
            action='store_false',
            dest='pull',
            default=not defaults.pull,
            help=(
                SUPPRESS if not defaults.pull else
                f'Do not pull newer docker image. Currently: {defaults.pull}'
            )
        )

        self.release.add_argument(
            '--release', '-r',
            default=defaults.release,
            choices=choices.release,
            help=f'ROS release to build. Currently: {defaults.release}',
        )

        class VolumesAction(Action):
            def __call__(
                    self,
                    _parser: ArgumentParser,
                    namespace: Namespace,
                    values: List[str],
                    _option_string: Optional[str] = None
            ) -> None:
                volumes = {}
                for value in values:
                    try:
                        host_path, container_path = value.split(':')
                    except ValueError:
                        host_path, container_path = value, value

                    host_path = os.path.realpath(os.path.expanduser(host_path))
                    container_path = os.path.realpath(os.path.expanduser(container_path))
                    volumes[host_path] = container_path

                setattr(namespace, self.dest, frozendict(volumes))

        self.volumes.add_argument(
            '--volumes', '-v',
            nargs='+',
            default=defaults.volumes,
            action=VolumesAction,
            help=f'Additional volumes to mount in docker container. Currently: {defaults.volumes}',
        )


flag = Flag()


rosdev_parser = ArgumentParser(parents=[flag.log_level])
rosdev_subparsers = rosdev_parser.add_subparsers(required=True)
rosdev_bash_parser = rosdev_subparsers.add_parser(
    'bash', parents=[
        flag.architecture,
        flag.build_num,
        flag.clean,
        flag.flavor,
        flag.gui,
        flag.log_level,
        flag.ports,
        flag.pull,
        flag.release,
        flag.volumes,
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
        flag.good_build_num,
        flag.good_release,
        flag.pull,
        flag.release,
    ]
)
rosdev_bisect_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.bisect').Bisect)
rosdev_clion_parser = rosdev_subparsers.add_parser(
    'clion',
    parents=[
        flag.architecture,
        flag.build_num,
        flag.pull,
        flag.release
    ]
)
rosdev_clion_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.clion').Clion)

rosdev_gen_parser = rosdev_subparsers.add_parser(
    'gen', parents=[])
rosdev_gen_subparsers = rosdev_gen_parser.add_subparsers(required=True)
rosdev_gen_colcon_parser = rosdev_gen_subparsers.add_parser(
    'colcon', parents=[])
rosdev_gen_colcon_subparsers = rosdev_gen_colcon_parser.add_subparsers(required=True)
rosdev_gen_colcon_build_parser = rosdev_gen_colcon_subparsers.add_parser(
    'build',
    parents=[
        flag.architecture,
        flag.asan,
        flag.build_num,
        flag.clean,
        flag.colcon_build_args,
        flag.debug,
        flag.pull,
        flag.release
    ])
rosdev_gen_colcon_build_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gen.colcon.build').Build)
rosdev_gen_clion_parser = rosdev_gen_subparsers.add_parser(
    'clion', parents=[]
)
rosdev_gen_clion_subparsers = rosdev_gen_clion_parser.add_subparsers(required=True)
rosdev_gen_clion_cmake_parser = rosdev_gen_clion_subparsers.add_parser(
    'cmake', parents=[]
)
rosdev_gen_clion_cmake_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gen.clion.cmake').Cmake)
rosdev_gen_clion_ide_parser = rosdev_gen_clion_subparsers.add_parser(
    'ide', parents=[]
)
rosdev_gen_clion_ide_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gen.clion.ide').Ide)
rosdev_gen_clion_toolchain_parser = rosdev_gen_clion_subparsers.add_parser(
    'toolchain',
    parents=[
        flag.architecture,
        flag.build_num,
        flag.ports,
        flag.pull,
        flag.release
    ]
)
rosdev_gen_clion_toolchain_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gen.clion.toolchain').Toolchain)
rosdev_gen_docker_parser = rosdev_gen_subparsers.add_parser(
    'docker', parents=[])
rosdev_gen_docker_subparsers = rosdev_gen_docker_parser.add_subparsers(required=True)
rosdev_gen_docker_container_parser = rosdev_gen_docker_subparsers.add_parser(
    'container',
    parents=[
        positional.command,
        flag.architecture,
        flag.clean,
        flag.interactive,
        flag.ports,
        flag.pull,
        flag.release,
        flag.volumes
    ]
)
rosdev_gen_docker_container_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gen.docker.container').Container)
rosdev_gen_global_parser = rosdev_gen_subparsers.add_parser(
    'global', parents=[]
)
rosdev_gen_global_subparsers = rosdev_gen_global_parser.add_subparsers(required=True)
rosdev_gen_global_defaults_parser = rosdev_gen_global_subparsers.add_parser(
    'defaults',
    parents=asdict(flag).values()
)
rosdev_gen_global_defaults_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gen.global.defaults').Defaults)
rosdev_gen_docker_image_parser = rosdev_gen_docker_subparsers.add_parser(
    'image', parents=[flag.architecture, flag.log_level, flag.pull, flag.release])
rosdev_gen_docker_image_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gen.docker.image').Image)
rosdev_gen_install_parser = rosdev_gen_subparsers.add_parser(
    'install', parents=[flag.architecture, flag.build_num, flag.log_level, flag.pull, flag.release])
rosdev_gen_install_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gen.install').Install)
rosdev_gen_local_parser = rosdev_gen_subparsers.add_parser(
    'local', parents=[]
)
rosdev_gen_local_subparsers = rosdev_gen_local_parser.add_subparsers(required=True)
rosdev_gen_local_defaults_parser = rosdev_gen_local_subparsers.add_parser(
    'defaults',
    parents=asdict(flag).values()
)
rosdev_gen_local_defaults_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gen.local.defaults').Defaults)
rosdev_gen_src_parser = rosdev_gen_subparsers.add_parser(
    'src',
    parents=[
        flag.build_num,
        flag.log_level,
        flag.pull,
        flag.release
    ]
)
rosdev_gen_src_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gen.src').Src)


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
    return handler(Options(**ChainMap(args.__dict__, asdict(defaults))))
