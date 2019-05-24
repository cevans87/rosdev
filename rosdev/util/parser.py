from __future__ import annotations
from argcomplete import autocomplete
from argparse import Action, ArgumentParser, Namespace, SUPPRESS
import ast
from collections import ChainMap
from dataclasses import asdict, dataclass, field
from frozendict import frozendict
from importlib import import_module
import logging
from pathlib import Path
from typing import Awaitable, Dict, FrozenSet, List, Optional
from uuid import UUID


@dataclass(frozen=True)
class Defaults:
    architecture: str = 'amd64'
    bad_build_num: Optional[int] = None
    bad_release: str = 'latest'
    build_num: Optional[int] = None
    build_type: str = 'Debug'
    ccache: bool = True
    colcon_build_args: Optional[str] = None
    command: Optional[str] = None
    executable: Optional[str] = None
    flavor: str = 'ros-core'
    global_setup: Optional[str] = '.rosdev/install/setup.bash'
    good_build_num: Optional[int] = None
    good_release: str = 'crystal'
    gui: bool = False
    interactive: bool = False
    local_setup: Optional[str] = None
    log_level: str = 'INFO'
    name: Optional[str] = None
    package: Optional[str] = None
    ports: FrozenSet[int] = frozenset()
    pull: bool = False
    release: str = 'latest'
    rosdep_install_args: Optional[str] = None
    sanitizer: Optional[str] = None
    uuid: Optional[str] = None
    volumes: frozendict = frozendict()

    @staticmethod
    def _get_local_overrides() -> Dict:
        # noinspection PyBroadException
        try:
            with open(f'{Path.cwd()}/.rosdev/defaults', 'r') as overrides_f_in:
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
    architecture = tuple(sorted({'amd64', 'arm32v7', 'arm64v8'}))
    bad_release = tuple(sorted({'bionic', 'crystal'}) + ['latest'])
    build_type = tuple(sorted({'Debug', 'MinSizeRel', 'Release', 'RelWithDebInfo'}))
    flavor = tuple(sorted({'desktop', 'desktop-full', 'ros-base' 'ros-core'}))
    good_release = tuple(sorted({'ardent', 'bionic', 'crystal'}))
    # noinspection PyProtectedMember
    log_level = tuple([name for _, name in sorted(logging._levelToName.items())])
    release = tuple(sorted({'ardent', 'bionic', 'crystal', 'kinetic', 'melodic'}) + ['latest'])
    sanitizer = tuple(sorted({'asan', 'lsan', 'msan', 'tsan', 'ubsan'}))


choices = Choices()


@dataclass(frozen=True)
class Flag:
    architecture: ArgumentParser = field(default_factory=gen_flag_parser)
    bad_build_num: ArgumentParser = field(default_factory=gen_flag_parser)
    bad_release: ArgumentParser = field(default_factory=gen_flag_parser)
    build_num: ArgumentParser = field(default_factory=gen_flag_parser)
    build_type: ArgumentParser = field(default_factory=gen_flag_parser)
    ccache: ArgumentParser = field(default_factory=gen_flag_parser)
    colcon_build_args: ArgumentParser = field(default_factory=gen_flag_parser)
    flavor: ArgumentParser = field(default_factory=gen_flag_parser)
    global_setup: ArgumentParser = field(default_factory=gen_flag_parser)
    good_build_num: ArgumentParser = field(default_factory=gen_flag_parser)
    good_release: ArgumentParser = field(default_factory=gen_flag_parser)
    gui: ArgumentParser = field(default_factory=gen_flag_parser)
    interactive: ArgumentParser = field(default_factory=gen_flag_parser)
    log_level: ArgumentParser = field(default_factory=gen_flag_parser)
    name: ArgumentParser = field(default_factory=gen_flag_parser)
    port: ArgumentParser = field(default_factory=gen_flag_parser)
    ports: ArgumentParser = field(default_factory=gen_flag_parser)
    pull: ArgumentParser = field(default_factory=gen_flag_parser)
    local_setup: ArgumentParser = field(default_factory=gen_flag_parser)
    release: ArgumentParser = field(default_factory=gen_flag_parser)
    rosdep_install_args: ArgumentParser = field(default_factory=gen_flag_parser)
    sanitizer: ArgumentParser = field(default_factory=gen_flag_parser)
    uuid: ArgumentParser = field(default_factory=gen_flag_parser)
    volumes: ArgumentParser = field(default_factory=gen_flag_parser)

    def __post_init__(self) -> None:
        self.architecture.add_argument(
            '--architecture', '-a',
            default=defaults.architecture,
            choices=choices.architecture,
            help=f'Architecture to build. Currently: {defaults.architecture}',
        )

        self.bad_release.add_argument(
            '--bad-release',
            default=defaults.bad_release,
            choices=choices.bad_release,
            help=f'Bad release to compare. Currently: {defaults.bad_release}'
        )

        # FIXME make mutually exclusive with --bad-build or combine flags
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

        self.build_type.add_argument(
            '--build-type',
            default=defaults.build_type,
            choices=choices.build_type,
            help=f'Build type. Currently: {defaults.build_type}',
        )

        ccache_group = self.ccache.add_mutually_exclusive_group()
        ccache_group.add_argument(
            '--ccache',
            action='store_true',
            default=defaults.ccache,
            help=(
                SUPPRESS if defaults.ccache else
                f'Enable ccache. Currently: {defaults.ccache}'
            )
        )
        ccache_group.add_argument(
            '--no-ccache',
            action='store_false',
            dest='ccache',
            default=defaults.ccache,
            help=(
                SUPPRESS if not defaults.ccache else
                f'Disable ccache. Currently: {defaults.ccache}'
            )
        )

        colcon_build_args_group = self.colcon_build_args.add_mutually_exclusive_group()
        colcon_build_args_group.add_argument(
            '--colcon-build-args',
            default=defaults.colcon_build_args,
            help=(
                f'Additional args to pass to colcon build. '
                f'Currently: {defaults.colcon_build_args}'
            )
        )
        colcon_build_args_group.add_argument(
            '--no-colcon-build-args',
            action='store_const',
            const=None,
            dest='colcon_build_args',
            default=defaults.colcon_build_args,
            help=(
                SUPPRESS if defaults.colcon_build_args is None else
                f'Do not pass additional args to colcon build. '
                f'Currently: {defaults.colcon_build_args}'
            )
        )

        self.flavor.add_argument(
            '--flavor',
            default=defaults.flavor,
            choices=choices.flavor,
            help=f'Linux flavor. Currently: {defaults.flavor}'
        )

        global_setup_group = self.global_setup.add_mutually_exclusive_group()
        global_setup_group.add_argument(
            '--global-setup',
            default=defaults.global_setup,
            help=(
                f'Path to global setup.bash file to source. '
                f'Currently: {defaults.global_setup}'
            )
        )
        global_setup_group.add_argument(
            '--no-global-setup',
            action='store_const',
            const=None,
            dest='global_setup',
            default=defaults.global_setup,
            help=(
                SUPPRESS if not defaults.global_setup else
                f'Do not source a global  setup.bash. '
                f'Currently: {defaults.global_setup}'
            )
        )

        # FIXME make mutually exclusive with --good-build or combine flags
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
            default=defaults.gui,
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
            default=defaults.interactive,
            help=(
                SUPPRESS if defaults.interactive else
                f'Make derived docker container interactive. Currently: {defaults.interactive}'
            )
        )
        interactive_group.add_argument(
            '--no-interactive',
            dest='interactive',
            action='store_false',
            default=defaults.interactive,
            help=(
                SUPPRESS if not defaults.interactive else
                f'Do not make derived docker container interactive. '
                f'Currently: {defaults.interactive}'
            )
        )

        local_setup_group = self.local_setup.add_mutually_exclusive_group()
        local_setup_group.add_argument(
            '--local-setup',
            default=defaults.local_setup,
            help=(
                f'Path to local setup.bash file to source. '
                f'Currently: {defaults.local_setup}'
            )
        )
        local_setup_group.add_argument(
            '--no-local-setup',
            action='store_const',
            const=None,
            dest='local_setup',
            default=defaults.local_setup,
            help=(
                SUPPRESS if not defaults.local_setup else
                f'Do not source a local setup.bash. '
                f'Currently: {defaults.local_setup}'
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
                f'Name to assign to docker container. '
                f'Existing container with name will be removed. '
                f'Currently: {defaults.name}'
            ),
        )

        # TODO add PortsAction to allow <host>:<container> mapping. See VolumesAction for reference.
        self.ports.add_argument(
            '--ports', '-p',
            nargs='*',
            type=int,
            default=sorted(defaults.ports),
            help=f'List of ports to expose in docker container. '
            f'Currently: {defaults.ports}'
        )
        pull_group = self.pull.add_mutually_exclusive_group()
        pull_group.add_argument(
            '--pull',
            action='store_true',
            default=defaults.pull,
            help=(
                SUPPRESS if defaults.pull else
                f'Pull newer docker image. '
                f'Currently: {defaults.pull}'
            )
        )
        pull_group.add_argument(
            '--no-pull',
            action='store_false',
            dest='pull',
            default=defaults.pull,
            help=(
                SUPPRESS if not defaults.pull else
                f'Do not pull newer docker image. '
                f'Currently: {defaults.pull}'
            )
        )

        self.release.add_argument(
            '--release', '-r',
            default=defaults.release,
            choices=choices.release,
            help=(
                f'ROS release to build. '
                f'Currently: {defaults.release}'
            )
        )

        rosdep_install_args_group = self.rosdep_install_args.add_mutually_exclusive_group()
        rosdep_install_args_group.add_argument(
            '--rosdep-install-args',
            default=defaults.rosdep_install_args,
            help=(
                f'Additional args to pass to rosdep install. '
                f'Currently: {defaults.rosdep_install_args}'
            )
        )
        rosdep_install_args_group.add_argument(
            '--no-rosdep-install-args',
            action='store_const',
            const=None,
            dest='rosdep_install_args',
            default=defaults.rosdep_install_args,
            help=(
                SUPPRESS if defaults.rosdep_install_args is None else
                f'Do not pass additional args to rosdep install. '
                f'Currently: {defaults.rosdep_install_args}'
            )
        )

        sanitizer_group = self.sanitizer.add_mutually_exclusive_group()
        sanitizer_group.add_argument(
            '--sanitizer',
            default=defaults.sanitizer,
            choices=choices.sanitizer,
            help=f'Build with sanitizer enabled. Currently: {defaults.sanitizer}',
        )
        sanitizer_group.add_argument(
            '--no-sanitizer',
            action='store_const',
            const=None,
            dest='sanitizer',
            default=defaults.sanitizer,
            help=(
                SUPPRESS if defaults.sanitizer is None else
                f'Do not build with a sanitizer enabled. '
                f'Currently: {defaults.sanitizer}'
            )
        )

        class UuidAction(Action):
            def __call__(
                    self,
                    _parser: ArgumentParser,
                    namespace: Namespace,
                    values: List[str],
                    _option_string: Optional[str] = None
            ) -> None:
                setattr(namespace, self.dest, f'{UUID(values[0])}')

        self.uuid.add_argument(
            '--uuid',
            default=defaults.uuid,
            action=UuidAction,
            type=str,
            help=(
                f'UUID to use for generated settings. '
                f' Currently: {defaults.uuid}'
            )
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


# TODO turn this parser tree into something like a defaultdict. Boilerplate to add a new parser
#  is tedious and error-prone.
rosdev_parser = ArgumentParser(parents=[flag.log_level])
rosdev_subparsers = rosdev_parser.add_subparsers(required=True)
rosdev_bash_parser = rosdev_subparsers.add_parser(
    'bash', parents=[
        flag.architecture,
        flag.build_num,
        flag.ccache,
        flag.flavor,
        flag.global_setup,
        flag.gui,
        flag.local_setup,
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
        flag.bad_release,
        flag.bad_build_num,
        flag.build_type,
        flag.colcon_build_args,
        flag.good_build_num,
        flag.good_release,
        flag.pull,
        flag.release,
        flag.sanitizer,
    ]
)
rosdev_bisect_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.bisect').Bisect)
rosdev_clion_parser = rosdev_subparsers.add_parser(
    'clion',
    parents=[
        flag.architecture,
        flag.build_num,
        flag.build_type,
        flag.global_setup,
        flag.local_setup,
        flag.pull,
        flag.release,
        flag.sanitizer,
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
        flag.build_num,
        flag.build_type,
        flag.colcon_build_args,
        flag.global_setup,
        flag.local_setup,
        flag.pull,
        flag.release,
        flag.sanitizer,
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
rosdev_gen_colcon_config_parser = rosdev_gen_colcon_subparsers.add_parser(
    'config',
    parents=[])
rosdev_gen_colcon_config_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gen.colcon.config').Config)
rosdev_gen_clion_ide_parser = rosdev_gen_clion_subparsers.add_parser(
    'ide', parents=[]
)
rosdev_gen_clion_ide_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gen.clion.ide').Ide)
rosdev_gen_clion_keepass_parser = rosdev_gen_clion_subparsers.add_parser(
    'keepass', parents=[]
)
rosdev_gen_clion_keepass_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gen.clion.keepass').Keepass)
rosdev_gen_clion_overlay_parser = rosdev_gen_clion_subparsers.add_parser(
    'overlay', parents=[]
)
rosdev_gen_clion_overlay_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gen.clion.overlay').Overlay)
rosdev_gen_clion_settings_parser = rosdev_gen_clion_subparsers.add_parser(
    'settings', parents=[]
)
rosdev_gen_clion_settings_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gen.clion.settings').Settings)
rosdev_gen_clion_toolchain_parser = rosdev_gen_clion_subparsers.add_parser(
    'toolchain',
    parents=[
        flag.architecture,
        flag.build_num,
        flag.global_setup,
        flag.local_setup,
        flag.ports,
        flag.pull,
        flag.release,
        flag.sanitizer,
    ]
)
rosdev_gen_clion_toolchain_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gen.clion.toolchain').Toolchain)
rosdev_gen_defaults_parser = rosdev_gen_subparsers.add_parser(
    'defaults',
    parents=asdict(flag).values()
)
rosdev_gen_defaults_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gen.defaults').Defaults)
rosdev_gen_docker_parser = rosdev_gen_subparsers.add_parser(
    'docker', parents=[])
rosdev_gen_docker_subparsers = rosdev_gen_docker_parser.add_subparsers(required=True)
rosdev_gen_docker_container_parser = rosdev_gen_docker_subparsers.add_parser(
    'container',
    parents=[
        positional.command,
        flag.architecture,
        flag.global_setup,
        flag.interactive,
        flag.local_setup,
        flag.ports,
        flag.pull,
        flag.release,
        flag.volumes
    ]
)
rosdev_gen_docker_container_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gen.docker.container').Container)
rosdev_gen_docker_image_parser = rosdev_gen_docker_subparsers.add_parser(
    'image', parents=[flag.architecture, flag.log_level, flag.pull, flag.release])
rosdev_gen_docker_image_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gen.docker.image').Image)
rosdev_gen_install_parser = rosdev_gen_subparsers.add_parser(
    'install', parents=[
        flag.architecture,
        flag.build_num,
        flag.log_level,
        flag.pull,
        flag.release
    ]
)
rosdev_gen_install_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gen.install').Install)
rosdev_gen_rosdep_parser = rosdev_gen_subparsers.add_parser(
    'rosdep', parents=[])
rosdev_gen_rosdep_subparsers = rosdev_gen_rosdep_parser.add_subparsers(required=True)
rosdev_gen_rosdep_config_parser = rosdev_gen_rosdep_subparsers.add_parser(
    'config',
    parents=[]
)
rosdev_gen_rosdep_install_parser = rosdev_gen_rosdep_subparsers.add_parser(
    'install',
    parents=[
        flag.architecture,
        flag.log_level,
        flag.pull,
        flag.release,
        flag.rosdep_install_args,
    ])
rosdev_gen_rosdep_install_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gen.rosdep.install').Install)
rosdev_gen_rosdev_parser = rosdev_gen_subparsers.add_parser(
    'rosdev', parents=[])
rosdev_gen_rosdev_subparsers = rosdev_gen_rosdev_parser.add_subparsers(required=True)
rosdev_gen_rosdev_config_parser = rosdev_gen_rosdev_subparsers.add_parser(
    'config',
    parents=[]
)
rosdev_gen_rosdev_config_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gen.rosdev.config').Config)
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
