from __future__ import annotations
from argcomplete import autocomplete
# noinspection PyProtectedMember
from argparse import (
    Action, ArgumentParser, _HelpAction, Namespace, _SubParsersAction, SUPPRESS
)
import ast
from collections import ChainMap
from dataclasses import asdict, dataclass, field, replace
from frozendict import frozendict
from importlib import import_module
import logging
from pathlib import Path
from typing import (
    AbstractSet, Awaitable, FrozenSet, List, Mapping, Optional, Sequence, Tuple,
    Union
)
from uuid import UUID

from rosdev.util.options import Options


def get_options_with_overrides() -> Options:
    overrides = []
    for path in [Path.cwd(), *Path.cwd().parents]:
        try:
            with open(f'{path}/.rosdev/overrides', 'r') as overrides_f_in:
                overrides.append(ast.literal_eval(overrides_f_in.read()))
        except FileNotFoundError:
            pass

    return Options(**ChainMap(*overrides))


options = get_options_with_overrides()


def gen_flag_parser() -> ArgumentParser:
    return ArgumentParser(add_help=False, conflict_handler='resolve')


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
    build_type = tuple(sorted({'Debug', 'MinSizeRel', 'Release', 'RelWithDebInfo'}))
    flavor = tuple(sorted({'desktop', 'desktop-full', 'ros-base' 'ros-core'}))
    # noinspection PyProtectedMember
    log_level = tuple([name for _, name in sorted(logging._levelToName.items())])
    ros_release = tuple(
        sorted({'ardent', 'bionic', 'crystal', 'dashing', 'kinetic', 'melodic'}) + ['latest']
    )
    sanitizer = tuple(sorted({'asan', 'lsan', 'msan', 'tsan', 'ubsan'}))


choices = Choices()


@dataclass(frozen=True)
class Flag:
    architecture: ArgumentParser = field(default_factory=gen_flag_parser)
    bad_build_num: ArgumentParser = field(default_factory=gen_flag_parser)
    build_type: ArgumentParser = field(default_factory=gen_flag_parser)
    ros_build_num: ArgumentParser = field(default_factory=gen_flag_parser)
    colcon_build_args: ArgumentParser = field(default_factory=gen_flag_parser)
    docker_container_name: ArgumentParser = field(default_factory=gen_flag_parser)
    enable_ccache: ArgumentParser = field(default_factory=gen_flag_parser)
    flavor: ArgumentParser = field(default_factory=gen_flag_parser)
    #global_setup: ArgumentParser = field(default_factory=gen_flag_parser)
    good_build_num: ArgumentParser = field(default_factory=gen_flag_parser)
    enable_gui: ArgumentParser = field(default_factory=gen_flag_parser)
    help: ArgumentParser = field(default_factory=gen_flag_parser)
    interactive_docker_container: ArgumentParser = field(default_factory=gen_flag_parser)
    log_level: ArgumentParser = field(default_factory=gen_flag_parser)
    ports: ArgumentParser = field(default_factory=gen_flag_parser)
    pull_docker_image: ArgumentParser = field(default_factory=gen_flag_parser)
    pull_ros_install: ArgumentParser = field(default_factory=gen_flag_parser)
    pull_ros_src: ArgumentParser = field(default_factory=gen_flag_parser)
    #local_setup: ArgumentParser = field(default_factory=gen_flag_parser)
    release: ArgumentParser = field(default_factory=gen_flag_parser)
    replace_docker_container: ArgumentParser = field(default_factory=gen_flag_parser)
    rosdep_install_args: ArgumentParser = field(default_factory=gen_flag_parser)
    sanitizer: ArgumentParser = field(default_factory=gen_flag_parser)
    uuid: ArgumentParser = field(default_factory=gen_flag_parser)
    volumes: ArgumentParser = field(default_factory=gen_flag_parser)

    def __post_init__(self) -> None:
        self.architecture.add_argument(
            '--architecture', '-a',
            default=options.architecture,
            choices=choices.architecture,
            help=f'Architecture to build. Currently: {options.architecture}',
        )

        # FIXME make mutually exclusive with --bad-build or combine flags
        self.bad_build_num.add_argument(
            '--bad-build-num',
            type=int,
            default=options.bad_build_num,
            help=f'Bad build number to compare. Supersedes --bad-build'
        )

        self.build_type.add_argument(
            '--build-type',
            default=options.build_type,
            choices=choices.build_type,
            help=f'Build type. Currently: {options.build_type}',
        )

        colcon_build_args_group = self.colcon_build_args.add_mutually_exclusive_group()
        colcon_build_args_group.add_argument(
            '--colcon-build-args',
            default=options.colcon_build_args,
            help=(
                f'Additional args to pass to colcon build. '
                f'Currently: {options.colcon_build_args}'
            )
        )
        colcon_build_args_group.add_argument(
            '--no-colcon-build-args',
            action='store_const',
            const=None,
            dest='colcon_build_args',
            default=options.colcon_build_args,
            help=(
                SUPPRESS if options.colcon_build_args is None else
                f'Do not pass additional args to colcon build. '
                f'Currently: {options.colcon_build_args}'
            )
        )

        self.docker_container_name.add_argument(
            '--docker-container-name',
            default=options.docker_container_name,
            help=(
                f'Name to assign to docker container. '
                f'Existing container with name will be removed. '
                f'Currently: {options.docker_container_name}'
            ),
        )

        enable_ccache_group = self.enable_ccache.add_mutually_exclusive_group()
        enable_ccache_group.add_argument(
            '--enable-ccache',
            action='store_true',
            default=options.enable_ccache,
            help=(
                SUPPRESS if options.enable_ccache else
                f'Enable ccache. Currently: {options.enable_ccache}'
            )
        )
        enable_ccache_group.add_argument(
            '--disable-ccache',
            action='store_false',
            dest='enable_ccache',
            default=options.enable_ccache,
            help=(
                SUPPRESS if not options.enable_ccache else
                f'Disable ccache. Currently: {options.enable_ccache}'
            )
        )

        self.flavor.add_argument(
            '--flavor',
            default=options.flavor,
            choices=choices.flavor,
            help=f'Linux flavor. Currently: {options.flavor}'
        )

        # FIXME make mutually exclusive with --good-build or combine flags
        self.good_build_num.add_argument(
            '--good-build-num',
            type=int,
            default=options.good_build_num,
            help=f'Good build number to compare. Supersedes --good-build'
        )

        enable_gui_group = self.enable_gui.add_mutually_exclusive_group()
        enable_gui_group.add_argument(
            '--enable-gui',
            default=options.enable_gui,
            action='store_true',
            help=(
                SUPPRESS if options.enable_gui else
                f'Allow container to use host X11 server. Currently: {options.enable_gui}'
            )
        )
        enable_gui_group.add_argument(
            '--disable-gui',
            dest='enable_gui',
            default=options.enable_gui,
            action='store_false',
            help=(
                SUPPRESS if not options.enable_gui else
                f'Do not allow container to use host X11 server. Currently: {options.enable_gui}'
            )
        )

        class HelpAction(_HelpAction):

            def __call__(
                    self,
                    _parser: ArgumentParser,
                    namespace,
                    values,
                    option_string=None
            ) -> None:
                module = import_module(namespace.rosdev_handler_module)
                if hasattr(namespace, 'rosdev_handler_class'):
                    _parser.description = getattr(
                        module,
                        namespace.rosdev_handler_class
                    ).__doc__
                else:
                    _parser.description = module.__doc__

                super().__call__(
                    parser=_parser,
                    namespace=namespace,
                    values=values,
                    option_string=option_string
                )

        self.help.add_argument(
            '--help', '-h',
            action=HelpAction,
            dest=SUPPRESS,
            help='show this help message and exit',
        )

        interactive_docker_container_group = (
            self.interactive_docker_container.add_mutually_exclusive_group()
        )
        interactive_docker_container_group.add_argument(
            '--interactive-docker-container',
            action='store_true',
            default=options.interactive_docker_container,
            help=(
                SUPPRESS if options.interactive_docker_container else
                f'Make derived docker container interactive. '
                f'Currently: {options.interactive_docker_container}'
            )
        )
        interactive_docker_container_group.add_argument(
            '--no-interactive-docker-container',
            dest='interactive_docker_container',
            action='store_false',
            default=options.interactive_docker_container,
            help=(
                SUPPRESS if not options.interactive_docker_container else
                f'Do not make derived docker container interactive. '
                f'Currently: {options.interactive_docker_container}'
            )
        )

        #local_setup_group = self.local_setup.add_mutually_exclusive_group()
        #local_setup_group.add_argument(
        #    '--local-setup',
        #    default=options.local_setup,
        #    help=(
        #        f'Path to local setup.bash file to source. '
        #        f'Currently: {options.local_setup}'
        #    )
        #)
        #local_setup_group.add_argument(
        #    '--no-local-setup',
        #    action='store_const',
        #    const=None,
        #    dest='local_setup',
        #    default=options.local_setup,
        #    help=(
        #        SUPPRESS if not options.local_setup else
        #        f'Do not source a local setup.bash. '
        #        f'Currently: {options.local_setup}'
        #    )
        #)

        self.log_level.add_argument(
            '--log-level',
            default=options.log_level,
            choices=choices.log_level,
            help=f'Currently: {options.log_level}',
        )

        # TODO add PortsAction to allow <host>:<container> mapping. See VolumesAction for reference.
        self.ports.add_argument(
            '--ports', '-p',
            nargs='*',
            type=int,
            default=sorted(options.ports),
            help=f'List of ports to expose in docker container. '
            f'Currently: {options.ports}'
        )
        pull_docker_image_group = self.pull_docker_image.add_mutually_exclusive_group()
        pull_docker_image_group.add_argument(
            '--pull-docker-image',
            action='store_true',
            default=options.pull_docker_image,
            help=(
                SUPPRESS if options.pull_docker_image else
                f'Pull newer docker image. '
                f'Currently: {options.pull_docker_image}'
            )
        )
        pull_docker_image_group.add_argument(
            '--no-pull-docker-image',
            action='store_false',
            dest='pull_docker_image',
            default=options.pull_docker_image,
            help=(
                SUPPRESS if not options.pull_docker_image else
                f'Do not pull newer docker image. '
                f'Currently: {options.pull_docker_image}'
            )
        )

        pull_ros_install_group = self.pull_ros_install.add_mutually_exclusive_group()
        pull_ros_install_group.add_argument(
            '--pull-ros-install',
            action='store_true',
            default=options.pull_ros_install,
            help=(
                SUPPRESS if options.pull_ros_install else
                f'Pull newer ros install. '
                f'Currently: {options.pull_ros_install}'
            )
        )
        pull_ros_install_group.add_argument(
            '--no-pull-ros-install',
            action='store_false',
            dest='pull_ros_install',
            default=options.pull_ros_install,
            help=(
                SUPPRESS if not options.pull_ros_install else
                f'Do not pull newer ros install. '
                f'Currently: {options.pull_ros_install}'
            )
        )

        pull_ros_src_group = self.pull_ros_src.add_mutually_exclusive_group()
        pull_ros_src_group.add_argument(
            '--pull-ros-src',
            action='store_true',
            default=options.pull_ros_src,
            help=(
                SUPPRESS if options.pull_ros_src else
                f'Pull newer ros src. '
                f'Currently: {options.pull_ros_src}'
            )
        )
        pull_ros_src_group.add_argument(
            '--no-pull-ros-src',
            action='store_false',
            dest='pull_ros_src',
            default=options.pull_ros_src,
            help=(
                SUPPRESS if not options.pull_ros_src else
                f'Do not pull newer ros src. '
                f'Currently: {options.pull_ros_src}'
            )
        )

        self.ros_build_num.add_argument(
            '--ros-build-num',
            type=int,
            default=options.ros_build_num,
            help=(
                f'Use specified build from OSRF build farm instead of {options.ros_release}. '
                f'Currently: {options.ros_build_num}'
            ),
        )

        self.release.add_argument(
            '--ros-release', '-r', '--release',
            default=options.ros_release,
            choices=choices.ros_release,
            help=(
                f'ROS release to build. '
                f'Currently: {options.ros_release}'
            )
        )

        replace_docker_container_group = (
            self.replace_docker_container.add_mutually_exclusive_group()
        )
        replace_docker_container_group.add_argument(
            '--replace-docker-container',
            action='store_true',
            default=options.replace_docker_container,
            help=(
                SUPPRESS if options.replace_docker_container else
                f'Replace docker containers that should only need to be built once. '
                f'Currently: {options.replace_docker_container}'
            )
        )
        replace_docker_container_group.add_argument(
            '--no-replace-docker-container',
            action='store_false',
            dest='replace_docker_container',
            default=options.replace_docker_container,
            help=(
                SUPPRESS if not options.replace_docker_container else
                f'Do not replace docker containers that should only need to be built once. '
                f'Currently: {options.replace_docker_container}'
            )
        )

        rosdep_install_args_group = self.rosdep_install_args.add_mutually_exclusive_group()
        rosdep_install_args_group.add_argument(
            '--rosdep-install-args',
            default=options.rosdep_install_args,
            help=(
                f'Additional args to pass to rosdep install. '
                f'Currently: {options.rosdep_install_args}'
            )
        )
        rosdep_install_args_group.add_argument(
            '--no-rosdep-install-args',
            action='store_const',
            const=None,
            dest='rosdep_install_args',
            default=options.rosdep_install_args,
            help=(
                SUPPRESS if options.rosdep_install_args is None else
                f'Do not pass additional args to rosdep install. '
                f'Currently: {options.rosdep_install_args}'
            )
        )

        sanitizer_group = self.sanitizer.add_mutually_exclusive_group()
        sanitizer_group.add_argument(
            '--sanitizer',
            default=options.sanitizer,
            choices=choices.sanitizer,
            help=f'Build with sanitizer enabled. Currently: {options.sanitizer}',
        )
        sanitizer_group.add_argument(
            '--no-sanitizer',
            action='store_const',
            const=None,
            dest='sanitizer',
            default=options.sanitizer,
            help=(
                SUPPRESS if options.sanitizer is None else
                f'Do not build with a sanitizer enabled. '
                f'Currently: {options.sanitizer}'
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
            default=options.uuid,
            action=UuidAction,
            type=str,
            help=(
                f'UUID to use for generated settings. '
                f' Currently: {options.uuid}'
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
            '--docker-container-volumes', '-v',
            nargs='+',
            default=options.docker_container_volumes,
            action=VolumesAction,
            help=(
                f'Additional volumes to mount in docker container. '
                f'Currently: {options.docker_container_volumes}'
            )
        )


flag = Flag()


@dataclass(frozen=True)
class Parser:
    sub_command: str
    positionals: Tuple[ArgumentParser, ...] = tuple()
    flags: FrozenSet[ArgumentParser] = frozenset()
    sub_parser_by_sub_command: Mapping[str, Parser] = frozendict()

    # noinspection PyShadowingNames
    def merged_with(
            self,
            *,
            sub_commands: Union[str, Sequence[str]],
            positionals: Sequence[ArgumentParser] = tuple(),
            flags: AbstractSet[ArgumentParser] = frozenset(),
    ) -> Parser:
        if isinstance(sub_commands, str):
            sub_commands = tuple(sub_commands.split())

        if len(sub_commands) == 0:
            parser = Parser(
                sub_command=self.sub_command,
                positionals=tuple([*self.positionals, *positionals]),
                flags=frozenset(self.flags | flags),
                sub_parser_by_sub_command=self.sub_parser_by_sub_command,
            )
        else:
            sub_parser = self.sub_parser_by_sub_command.get(sub_commands[0])
            if sub_parser is None:
                sub_parser = Parser(sub_command=sub_commands[0])

            sub_parser = sub_parser.merged_with(
                sub_commands=sub_commands[1:],
                positionals=positionals,
                flags=flags,
            )

            parser = replace(
                self,
                sub_parser_by_sub_command=frozendict(
                    ChainMap(
                        {sub_parser.sub_command: sub_parser},
                        self.sub_parser_by_sub_command,
                    )
                )
            )

        return parser

    # noinspection PyShadowingNames
    def get_argument_parser(
            self,
            parent_sub_argument_parser: Optional[_SubParsersAction] = None,
            parent_flags: AbstractSet[ArgumentParser] = frozenset(),
            prev_sub_commands: Sequence[str] = tuple()
    ) -> ArgumentParser:
        # noinspection PyProtectedMember
        flags = frozenset(
            {flag._actions[0].dest: flag for flag in parent_flags | self.flags}.values()
        )

        if parent_sub_argument_parser is None:
            argument_parser = ArgumentParser(add_help=False, conflict_handler='resolve')
        else:
            # noinspection PyProtectedMember
            argument_parser = parent_sub_argument_parser.add_parser(
                self.sub_command,
                add_help=False,
                conflict_handler='resolve',
                parents=[
                    *self.positionals,
                    *sorted(flags, key=lambda flag: flag._actions[0].dest),
                ],
            )

        rosdev_handler_module = '.'.join([*prev_sub_commands, self.sub_command])
        argument_parser.set_defaults(rosdev_handler_module=rosdev_handler_module)

        if not self.sub_parser_by_sub_command:
            rosdev_handler_class = ''.join(
                sub_command.capitalize() for sub_command in
                [*prev_sub_commands[1:], self.sub_command]
            )
            argument_parser.set_defaults(rosdev_handler_class=rosdev_handler_class)
        else:
            sub_argument_parser = argument_parser.add_subparsers(required=True)
            for sub_parser in self.sub_parser_by_sub_command.values():
                sub_parser.get_argument_parser(
                    parent_sub_argument_parser=sub_argument_parser,
                    parent_flags=flags,
                    prev_sub_commands=tuple([*prev_sub_commands, self.sub_command])
                )

        return argument_parser


parser = Parser(
    sub_command='rosdev',
    flags=frozenset({
        flag.help,
        flag.log_level,
    })
)

parser = parser.merged_with(
    sub_commands='bash',
    flags=frozenset({
        flag.architecture,
        flag.enable_ccache,
        flag.enable_gui,
        flag.flavor,
        #flag.global_setup,
        #flag.local_setup,
        flag.ports,
        flag.pull_docker_image,
        flag.release,
        flag.replace_docker_container,
        flag.ros_build_num,
        flag.volumes,
    }),
)

parser = parser.merged_with(
    sub_commands='bisect',
    positionals=(
        positional.command,
    ),
    flags=frozenset({
        flag.architecture,
        flag.bad_build_num,
        flag.build_type,
        flag.colcon_build_args,
        flag.good_build_num,
        flag.pull_docker_image,
        flag.release,
        flag.sanitizer,
    })
)

parser = parser.merged_with(
    sub_commands='clion',
    flags=frozenset({
        flag.architecture,
        flag.ros_build_num,
        flag.build_type,
        #flag.global_setup,
        #flag.local_setup,
        flag.pull_docker_image,
        flag.release,
        flag.sanitizer,
    }),
)

parser = parser.merged_with(
    sub_commands='gen architecture',
    flags=frozenset({
        flag.architecture
    })
)

parser = parser.merged_with(
    sub_commands='gen colcon build',
    flags=frozenset({
        flag.architecture,
        flag.build_type,
        flag.colcon_build_args,
        #flag.global_setup,
        #flag.local_setup,
        flag.pull_docker_image,
        flag.release,
        flag.ros_build_num,
        flag.sanitizer,
    })
)

parser = parser.merged_with(sub_commands='gen clion cmake')
parser = parser.merged_with(sub_commands='gen clion config')
parser = parser.merged_with(sub_commands='gen clion ide')
parser = parser.merged_with(sub_commands='gen clion keepass')
# parser = parser.merged_with(sub_commands='gen clion overlay')
parser = parser.merged_with(sub_commands='gen clion settings')
parser = parser.merged_with(sub_commands='gen clion toolchain')

parser = parser.merged_with(
    sub_commands='gen docker container',
    positionals=(
        positional.command,
    ),
    flags=frozenset({
        flag.architecture,
        #flag.global_setup,
        flag.interactive_docker_container,
        #flag.local_setup,
        flag.ports,
        flag.pull_docker_image,
        flag.release,
        flag.replace_docker_container,
        flag.volumes,
    })
)

parser = parser.merged_with(
    sub_commands='gen docker image',
    flags=frozenset({
        flag.architecture,
        flag.pull_docker_image,
        flag.release,
    })
)

parser = parser.merged_with(
    sub_commands='gen overrides',
    flags=frozenset(asdict(flag).values())
)

parser = parser.merged_with(sub_commands='gen pycharm config')
parser = parser.merged_with(sub_commands='gen pycharm ide')
parser = parser.merged_with(sub_commands='gen pycharm keepass')
# parser = parser.merged_with(sub_commands='gen pycharm overlay')
parser = parser.merged_with(sub_commands='gen pycharm settings')

parser = parser.merged_with(
    sub_commands='gen ros build num',
    flags=frozenset({
        flag.architecture,
        flag.ros_build_num,
        flag.release,
    })
)

parser = parser.merged_with(
    sub_commands='gen ros install',
    flags=frozenset({
        flag.architecture,
        flag.pull_docker_image,
        flag.release,
        flag.ros_build_num,
    })
)

parser = parser.merged_with(
    sub_commands='gen ros src',
    flags=frozenset({
        flag.architecture,
        flag.pull_docker_image,
        flag.release,
        flag.ros_build_num,
    })
)

parser = parser.merged_with(
    sub_commands='gen rosdep config',
)

parser = parser.merged_with(
    sub_commands='gen rosdep install',
    flags=frozenset({
        flag.architecture,
        flag.pull_docker_image,
        flag.release,
        flag.rosdep_install_args,
    })
)

parser = parser.merged_with(
    sub_commands='gen rosdev',
)

parser = parser.merged_with(
    sub_commands='gen src',
    flags=frozenset({
        flag.pull_docker_image,
        flag.release,
        flag.ros_build_num,
    })
)

parser = parser.merged_with(
    sub_commands='pycharm',
    flags=frozenset({
        flag.architecture,
        flag.build_type,
        #flag.global_setup,
        #flag.local_setup,
        flag.pull_docker_image,
        flag.release,
        flag.ros_build_num,
        flag.sanitizer,
    })
)


def get_handler_and_options(args: Optional[List[str]]) -> (Awaitable, Options):
    argument_parser = parser.get_argument_parser()
    autocomplete(argument_parser)

    import sys
    args = args if args is not None else sys.argv[1:]
    # noinspection PyBroadException
    try:
        args = argument_parser.parse_args(args)
    except Exception:
        args.append('-h')
        argument_parser.parse_args(args)

    for k, v in args.__dict__.items():
        if isinstance(v, list):
            setattr(args, k, frozenset(v))

    handler = getattr(
        import_module(args.rosdev_handler_module),
        args.rosdev_handler_class
    )
    del args.__dict__['rosdev_handler_module']
    del args.__dict__['rosdev_handler_class']

    global options
    options = replace(options, **args.__dict__)
    import asyncio
    options = asyncio.run(handler._resolve_all_options(options))

    return handler.run(replace(options, **args.__dict__)), options
