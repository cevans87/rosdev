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
from stringcase import capitalcase
from typing import (
    AbstractSet, Awaitable, Dict, FrozenSet, List, Mapping, Optional, Sequence, Tuple,
    Union
)
from uuid import UUID


@dataclass(frozen=True)
class Defaults:
    architecture: str = 'amd64'
    bad_build_num: Optional[int] = None
    build_num: Optional[int] = None
    build_type: str = 'Debug'
    ccache: bool = True
    colcon_build_args: Optional[str] = None
    command: Optional[str] = None
    container_name: Optional[str] = None
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
    pull: bool = False
    release: str = 'latest'
    # TODO change this to replace_docker_container
    replace_named_container: bool = True
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
    release = tuple(
        sorted({'ardent', 'bionic', 'crystal', 'dashing', 'kinetic', 'melodic'}) + ['latest']
    )
    sanitizer = tuple(sorted({'asan', 'lsan', 'msan', 'tsan', 'ubsan'}))


choices = Choices()


@dataclass(frozen=True)
class Flag:
    architecture: ArgumentParser = field(default_factory=gen_flag_parser)
    bad_build_num: ArgumentParser = field(default_factory=gen_flag_parser)
    build_num: ArgumentParser = field(default_factory=gen_flag_parser)
    build_type: ArgumentParser = field(default_factory=gen_flag_parser)
    ccache: ArgumentParser = field(default_factory=gen_flag_parser)
    colcon_build_args: ArgumentParser = field(default_factory=gen_flag_parser)
    container_name: ArgumentParser = field(default_factory=gen_flag_parser)
    flavor: ArgumentParser = field(default_factory=gen_flag_parser)
    global_setup: ArgumentParser = field(default_factory=gen_flag_parser)
    good_build_num: ArgumentParser = field(default_factory=gen_flag_parser)
    gui: ArgumentParser = field(default_factory=gen_flag_parser)
    help: ArgumentParser = field(default_factory=gen_flag_parser)
    interactive: ArgumentParser = field(default_factory=gen_flag_parser)
    log_level: ArgumentParser = field(default_factory=gen_flag_parser)
    ports: ArgumentParser = field(default_factory=gen_flag_parser)
    pull: ArgumentParser = field(default_factory=gen_flag_parser)
    local_setup: ArgumentParser = field(default_factory=gen_flag_parser)
    release: ArgumentParser = field(default_factory=gen_flag_parser)
    replace_named_container: ArgumentParser = field(default_factory=gen_flag_parser)
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

        self.container_name.add_argument(
            '--container-name',
            default=defaults.container_name,
            help=(
                f'Name to assign to docker container. '
                f'Existing container with name will be removed. '
                f'Currently: {defaults.container_name}'
            ),
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
                    _parser.description = getattr(module, 'rosdev_handler_class').__doc__
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

        self.replace_named_container.add_argument(
            '--replace-named-container',
            default=defaults.replace_named_container,
            help=(
                f'Replace named containers that should only need to be built once. '
                f'Currently: {defaults.replace_named_container}'
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
                sub_parser_by_sub_command=self.sub_parser_by_sub_command
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

        argument_parser.set_defaults(
            rosdev_handler_module='.'.join(tuple([*prev_sub_commands, self.sub_command])),
        )

        if not self.sub_parser_by_sub_command:
            argument_parser.set_defaults(
                rosdev_handler_class=capitalcase(self.sub_command),
            )
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
        flag.build_num,
        flag.ccache,
        flag.flavor,
        flag.global_setup,
        flag.gui,
        flag.local_setup,
        flag.ports,
        flag.pull,
        flag.release,
        flag.volumes,
    })
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
        flag.pull,
        flag.release,
        flag.sanitizer,
    })
)

parser = parser.merged_with(
    sub_commands='clion',
    flags=frozenset({
        flag.architecture,
        flag.build_num,
        flag.build_type,
        flag.global_setup,
        flag.local_setup,
        flag.pull,
        flag.release,
        flag.sanitizer,
    })
)

parser = parser.merged_with(
    sub_commands='gen colcon build',
    flags=frozenset({
        flag.architecture,
        flag.build_num,
        flag.build_type,
        flag.colcon_build_args,
        flag.global_setup,
        flag.local_setup,
        flag.pull,
        flag.release,
        flag.sanitizer,
    })
)

parser = parser.merged_with(sub_commands='gen clion cmake')
parser = parser.merged_with(sub_commands='gen clion config')
parser = parser.merged_with(sub_commands='gen clion ide')
parser = parser.merged_with(sub_commands='gen clion keepass')
parser = parser.merged_with(sub_commands='gen clion overlay')
parser = parser.merged_with(sub_commands='gen clion settings')
parser = parser.merged_with(sub_commands='gen clion toolchain')

parser = parser.merged_with(
    sub_commands='gen defaults',
    flags=frozenset(asdict(flag).values())
)

parser = parser.merged_with(
    sub_commands='gen docker container',
    positionals=(
        positional.command,
    ),
    flags=frozenset({
        flag.architecture,
        flag.global_setup,
        flag.interactive,
        flag.local_setup,
        flag.ports,
        flag.pull,
        flag.release,
        flag.volumes,
    })
)

parser = parser.merged_with(
    sub_commands='gen docker image',
    flags=frozenset({
        flag.architecture,
        flag.pull,
        flag.release,
    })
)

parser = parser.merged_with(
    sub_commands='gen install',
    flags=frozenset({
        flag.architecture,
        flag.build_num,
        flag.pull,
        flag.release
    })
)

parser = parser.merged_with(
    sub_commands='gen rosdep config',
)

parser = parser.merged_with(
    sub_commands='gen rosdep install',
    flags=frozenset({
        flag.architecture,
        flag.pull,
        flag.release,
        flag.rosdep_install_args,
    })
)

parser = parser.merged_with(
    sub_commands='gen rosdev config',
)

parser = parser.merged_with(
    sub_commands='gen src',
    flags=frozenset({
        flag.build_num,
        flag.pull,
        flag.release
    })
)


def get_handler(args: Optional[List[str]]) -> Awaitable:
    argument_parser = parser.get_argument_parser()
    autocomplete(argument_parser)

    import sys
    args = args if args is not None else sys.argv[1:]
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

    from rosdev.util.options import Options
    return handler(Options(**ChainMap(args.__dict__, asdict(defaults))))
