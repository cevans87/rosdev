from __future__ import annotations
from argcomplete import autocomplete
from atools import memoize
# noinspection PyProtectedMember
from argparse import Action, ArgumentParser, Namespace, _SubParsersAction, REMAINDER, SUPPRESS
from collections import ChainMap
from dataclasses import dataclass, field, InitVar, replace
import logging
from platform import machine
import sys
from typing import AbstractSet, Any, FrozenSet, List, Mapping, Optional, Sequence, Tuple, Union

from rosdev.util.frozendict import frozendict, FrozenDict
from rosdev.util.path import Path


memoize_db = memoize(db_path=Path.rosdev() / 'db')

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class Options:
    architecture: str = {'x86_64': 'amd64', 'arm': 'arm32v7', 'aarch64': 'arm64v8'}[machine()]
    backend_ssh_builder_identity_path: str = ''
    backend_ssh_builder_uri: str = ''
    backend_ssh_runner_identity_path: str = ''
    backend_ssh_runner_uri: str = ''
    build_type: str = 'Debug'
    colcon_build_args: str = ''
    docker_container_ccache: bool = True
    docker_container_gui: bool = False
    docker_container_ports: FrozenDict[int, int] = frozendict()  # container:host
    docker_container_replace: bool = False
    docker_container_volumes: FrozenDict[str, str] = frozendict()  # host:container
    docker_entrypoint_sh_quick_setup_overlay: bool = True
    docker_entrypoint_sh_quick_setup_underlay: bool = True
    docker_entrypoint_sh_setup_overlay: bool = True
    docker_entrypoint_sh_setup_underlay: bool = True
    docker_image_pull: bool = False
    docker_image_replace: bool = False
    dry_run: bool = False
    executable: str = ''
    handler_class: str = ''
    handler_module: str = ''
    help: str = ''
    idea_ide_name: str = ''
    idea_ide_path: str = ''
    idea_uuid: str = ''
    log_level: str = 'INFO'
    package: str = ''
    release: str = 'latest'
    remainder: str = ''
    reset_cache: bool = False
    run_main: bool = True
    run_validate_options: bool = True
    sanitizer: str = ''

    @staticmethod
    @memoize
    def of_args(args: Tuple[str] = tuple(sys.argv[1:])) -> Options:
        sticky_options = _get_sticky_options()

        parser = Parser.of_options(sticky_options)
        parsed_args = parser.parse_args(args)
        options = replace(sticky_options, **parsed_args)

        if (
                (sticky_options.architecture != options.architecture) or
                (sticky_options.release != options.release)
        ):
            swap_options = Options(architecture=options.architecture, release=options.release)
            swap_options = _get_secondary_sticky_options(swap_options)
            if 'backend_ssh_builder_identity_path' not in parsed_args:
                options = replace(
                    options,
                    backend_ssh_builder_identity_path=swap_options.backend_ssh_builder_identity_path
                )
            if 'backend_ssh_builder_uri' not in parsed_args:
                options = replace(
                    options,
                    backend_ssh_builder_uri=swap_options.backend_ssh_builder_uri
                )
            if 'backend_ssh_runner_identity_path' not in parsed_args:
                options = replace(
                    options,
                    backend_ssh_runner_identity_path=swap_options.backend_ssh_runner_identity_path
                )
            if 'backend_ssh_runner_uri' not in parsed_args:
                options = replace(
                    options,
                    backend_ssh_runner_uri=swap_options.backend_ssh_runner_uri
                )

        _set_sticky_options(options)

        return options


@memoize_db
def _get_primary_sticky_options() -> Options:
    return Options()


@memoize_db(keygen=lambda options: (options.architecture, options.release))
def _get_secondary_sticky_options(options: Options) -> Options:
    return options


def _get_sticky_options() -> Options:
    sticky_options = _get_primary_sticky_options()
    sticky_options = _get_secondary_sticky_options(sticky_options)

    return Options(
        architecture=sticky_options.architecture,
        backend_ssh_builder_identity_path=sticky_options.backend_ssh_builder_identity_path,
        backend_ssh_builder_uri=sticky_options.backend_ssh_builder_uri,
        backend_ssh_runner_identity_path=sticky_options.backend_ssh_runner_identity_path,
        backend_ssh_runner_uri=sticky_options.backend_ssh_runner_uri,
        release=sticky_options.release,
    )


def _set_sticky_options(sticky_options: Options) -> None:
    _get_primary_sticky_options.memoize.update()(sticky_options)
    _get_secondary_sticky_options.memoize.update(sticky_options)(sticky_options)


def gen_flag_parser() -> ArgumentParser:
    return ArgumentParser(add_help=False, conflict_handler='resolve')


@dataclass(frozen=True)
class Positional:
    command: ArgumentParser = field(default_factory=gen_flag_parser)
    executable: ArgumentParser = field(default_factory=gen_flag_parser)
    package: ArgumentParser = field(default_factory=gen_flag_parser)
    remainder: ArgumentParser = field(default_factory=gen_flag_parser)

    def __post_init__(self) -> None:
        self.command.add_argument('command')
        self.executable.add_argument('executable')
        self.package.add_argument('package')

        class RemainderAction(Action):
            def __call__(
                    self,
                    _parser: ArgumentParser,
                    namespace: Namespace,
                    values: List[str],
                    _option_string: Optional[str] = None
            ) -> None:
                setattr(namespace, self.dest, ' '.join(values))

        self.remainder.add_argument(
            'remainder',
            action=RemainderAction,
            nargs=REMAINDER,
        )


# noinspection PyUnusedLocal
def get_positional(options: Options) -> Positional:
    return Positional()


@dataclass(frozen=True)
class Choices:
    architecture = tuple(sorted({'amd64', 'arm32v7', 'arm64v8'}))
    build_type = tuple(sorted({'Debug', 'MinSizeRel', 'Release', 'RelWithDebInfo'}))
    idea_ide_name = tuple(sorted({'CLion', 'PyCharm'}))
    # noinspection PyProtectedMember
    log_level = tuple([name for _, name in sorted(logging._levelToName.items())])
    # TODO get release options dynamically, memoize, and invalidate on bad input?
    release = tuple(sorted({'dashing', 'eloquent', 'foxy', 'kinetic', 'melodic'}) + ['latest'])
    sanitizer = tuple(sorted({'asan', 'lsan', 'msan', 'tsan', 'ubsan'}))


# noinspection PyUnusedLocal
def get_choices(options: Options) -> Choices:
    return Choices()


@dataclass(frozen=True)
class Flag:
    options: InitVar[Options]

    architecture: ArgumentParser = field(default_factory=gen_flag_parser)
    backend_ssh_builder_uri: ArgumentParser = field(default_factory=gen_flag_parser)
    backend_ssh_builder_identity_path: ArgumentParser = field(default_factory=gen_flag_parser)
    backend_ssh_runner_uri: ArgumentParser = field(default_factory=gen_flag_parser)
    backend_ssh_runner_identity_path: ArgumentParser = field(default_factory=gen_flag_parser)
    build_type: ArgumentParser = field(default_factory=gen_flag_parser)
    colcon_build_args: ArgumentParser = field(default_factory=gen_flag_parser)
    docker_container_ccache: ArgumentParser = field(default_factory=gen_flag_parser)
    docker_container_gui: ArgumentParser = field(default_factory=gen_flag_parser)
    docker_container_ports: ArgumentParser = field(default_factory=gen_flag_parser)
    docker_container_replace: ArgumentParser = field(default_factory=gen_flag_parser)
    docker_container_volumes: ArgumentParser = field(default_factory=gen_flag_parser)
    docker_entrypoint_sh_setup_overlay: ArgumentParser = field(default_factory=gen_flag_parser)
    docker_entrypoint_sh_setup_underlay: ArgumentParser = field(default_factory=gen_flag_parser)
    docker_image_pull: ArgumentParser = field(default_factory=gen_flag_parser)
    docker_image_replace: ArgumentParser = field(default_factory=gen_flag_parser)
    dry_run: ArgumentParser = field(default_factory=gen_flag_parser)
    help: ArgumentParser = field(default_factory=gen_flag_parser)
    idea_ide_name: ArgumentParser = field(default_factory=gen_flag_parser)
    idea_uuid: ArgumentParser = field(default_factory=gen_flag_parser)
    log_level: ArgumentParser = field(default_factory=gen_flag_parser)
    release: ArgumentParser = field(default_factory=gen_flag_parser)
    reset_cache: ArgumentParser = field(default_factory=gen_flag_parser)
    run_main: ArgumentParser = field(default_factory=gen_flag_parser)
    run_validate_options: ArgumentParser = field(default_factory=gen_flag_parser)
    sanitizer: ArgumentParser = field(default_factory=gen_flag_parser)

    def __post_init__(self, options: Options) -> None:
        choices = get_choices(options)

        self.architecture.add_argument(
            '--architecture', '-a',
            default=options.architecture,
            choices=choices.architecture,
            help=f'Architecture to build. Currently: {options.architecture}',
        )

        backend_ssh_builder_identity_path_group = (
            self.backend_ssh_builder_identity_path.add_mutually_exclusive_group()
        )
        backend_ssh_builder_identity_path_group.add_argument(
            '--backend-ssh-builder-identity-path',
            default=options.backend_ssh_builder_identity_path,
            help=f'Identity to use when connecting to backend builder endpoint URI.'
                 f' Currently: {options.backend_ssh_builder_identity_path}'
        )
        backend_ssh_builder_identity_path_group.add_argument(
            '--no-backend-ssh-builder-identity-path',
            action='store_const',
            const='',
            dest='backend_ssh_builder_identity_path',
            default=options.backend_ssh_builder_identity_path,
            help=(
                SUPPRESS if options.backend_ssh_builder_identity_path is None else
                f'Do not use identity when connecting to backend builder endpoint URI.'
                f' Currently: {options.backend_ssh_builder_identity_path}'
            ),
        )

        backend_ssh_builder_uri_group = (
            self.backend_ssh_builder_uri.add_mutually_exclusive_group()
        )
        backend_ssh_builder_uri_group.add_argument(
            '--backend-ssh-builder-uri',
            default=options.backend_ssh_builder_uri,
            help=f'Backend builder endpoint URI.'
                 f' Currently: {options.backend_ssh_builder_uri}'
        )
        backend_ssh_builder_uri_group.add_argument(
            '--no-backend-ssh-builder-uri',
            action='store_const',
            const='',
            dest='backend_ssh_builder_uri',
            default=options.backend_ssh_builder_uri,
            help=(
                SUPPRESS if options.backend_ssh_builder_uri is None else
                f'Do not set backend builder endpoint URI.'
                f' Currently: {options.backend_ssh_builder_uri}'
            ),
        )

        backend_ssh_runner_identity_path_group = (
            self.backend_ssh_runner_identity_path.add_mutually_exclusive_group()
        )
        backend_ssh_runner_identity_path_group.add_argument(
            '--backend-ssh-runner-identity-path',
            default=options.backend_ssh_runner_identity_path,
            help=f'Identity to use when connecting to backend runner endpoint URI.'
                 f' Currently: {options.backend_ssh_runner_identity_path}'
        )
        backend_ssh_runner_identity_path_group.add_argument(
            '--no-backend-ssh-runner-identity-path',
            action='store_const',
            const='',
            dest='backend_ssh_runner_identity_path',
            default=options.backend_ssh_runner_identity_path,
            help=(
                SUPPRESS if options.backend_ssh_runner_identity_path is None else
                f'Do not use identity when connecting to backend runner endpoint URI.'
                f' Currently: {options.backend_ssh_runner_identity_path}'
            ),
        )

        backend_ssh_runner_uri_group = (
            self.backend_ssh_runner_uri.add_mutually_exclusive_group()
        )
        backend_ssh_runner_uri_group.add_argument(
            '--backend-ssh-runner-uri',
            default=options.backend_ssh_runner_uri,
            help=f'Backend runner endpoint URI.'
                 f' Currently: {options.backend_ssh_runner_uri}'
        )
        backend_ssh_runner_uri_group.add_argument(
            '--no-backend-ssh-runner-uri',
            action='store_const',
            const='',
            dest='backend_ssh_runner_uri',
            default=options.backend_ssh_runner_uri,
            help=(
                SUPPRESS if options.backend_ssh_runner_uri is None else
                f'Do not set backend runner endpoint URI.'
                f' Currently: {options.backend_ssh_runner_uri}'
            ),
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
                f'Additional args to pass to colcon build.'
                f' Currently: {options.colcon_build_args}'
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
                f'Do not pass additional args to colcon build.'
                f' Currently: {options.colcon_build_args}'
            )
        )

        docker_container_ccache_group = (
            self.docker_container_ccache.add_mutually_exclusive_group()
        )
        docker_container_ccache_group.add_argument(
            '--docker-container-ccache',
            action='store_true',
            default=options.docker_container_ccache,
            help=(
                SUPPRESS if options.docker_container_ccache else
                f'Enable ccache. Currently: {options.docker_container_ccache}'
            )
        )
        docker_container_ccache_group.add_argument(
            '--no-docker-container-ccache',
            action='store_false',
            dest='docker_container_ccache',
            default=options.docker_container_ccache,
            help=(
                SUPPRESS if not options.docker_container_ccache else
                f'Disable ccache. Currently: {options.docker_container_ccache}'
            )
        )

        docker_container_gui_group = self.docker_container_gui.add_mutually_exclusive_group()
        docker_container_gui_group.add_argument(
            '--docker-container-gui',
            default=options.docker_container_gui,
            action='store_true',
            help=(
                SUPPRESS if options.docker_container_gui else
                f'Allow container to use host X11 server.'
                f' Currently: {options.docker_container_gui}'
            )
        )
        docker_container_gui_group.add_argument(
            '--no-docker-container-gui',
            dest='docker_container_gui',
            default=options.docker_container_gui,
            action='store_false',
            help=(
                SUPPRESS if not options.docker_container_gui else
                f'Do not allow container to use host X11 server.'
                f' Currently: {options.docker_container_gui}'
            )
        )

        class DockerContainerPortsAction(Action):
            def __call__(
                    self,
                    _parser: ArgumentParser,
                    namespace: Namespace,
                    values: List[str],
                    _option_string: Optional[str] = None
            ) -> None:
                docker_container_ports = {}
                for value in values:
                    try:
                        container_port, host_port = value.split(':')
                    except ValueError:
                        container_port, host_port = value, value

                    container_port, host_port = int(container_port), int(host_port)

                    docker_container_ports[container_port] = host_port
                docker_container_ports = frozendict(docker_container_ports)

                setattr(namespace, self.dest, docker_container_ports)

        self.docker_container_ports.add_argument(
            '--docker-container-ports',
            nargs='*',
            default=options.docker_container_ports,
            action=DockerContainerPortsAction,
            help=f'List of ports to expose in docker container.'
                 f' Currently: {options.docker_container_ports}'
        )

        docker_container_replace_group = (
            self.docker_container_replace.add_mutually_exclusive_group()
        )
        docker_container_replace_group.add_argument(
            '--docker-container-replace',
            action='store_true',
            default=options.docker_container_replace,
            help=(
                SUPPRESS if options.docker_container_replace else
                f'Replace docker containers that should only need to be built once.'
                f' Currently: {options.docker_container_replace}'
            )
        )
        docker_container_replace_group.add_argument(
            '--no-docker-container-replace',
            action='store_false',
            dest='docker_container_replace',
            default=options.docker_container_replace,
            help=(
                SUPPRESS if not options.docker_container_replace else
                f'Do not replace docker containers that should only need to be built once.'
                f' Currently: {options.docker_container_replace}'
            )
        )

        class DockerContainerVolumesAction(Action):
            def __call__(
                    self,
                    _parser: ArgumentParser,
                    namespace: Namespace,
                    values: List[str],
                    _option_string: Optional[str] = None
            ) -> None:
                docker_container_volumes = {}
                for value in values:
                    try:
                        host_path, container_path = value.split(':')
                    except ValueError:
                        host_path, container_path = value, value

                    docker_container_volumes[host_path] = container_path
                docker_container_volumes = frozendict(docker_container_volumes)

                setattr(namespace, self.dest, docker_container_volumes)

        self.docker_container_volumes.add_argument(
            '--docker-container-volumes',
            nargs='*',
            default=options.docker_container_volumes,
            action=DockerContainerVolumesAction,
            help=(
                f'Additional volumes to mount in docker container.'
                f' Currently: {options.docker_container_volumes}'
            )
        )

        docker_entrypoint_sh_setup_overlay_group = (
            self.docker_entrypoint_sh_setup_overlay.add_mutually_exclusive_group()
        )
        docker_entrypoint_sh_setup_overlay_group.add_argument(
            '--docker-entrypoint-sh-setup-overlay',
            action='store_true',
            default=options.docker_entrypoint_sh_setup_overlay,
            help=(
                f'Source setup.bash from workspace install.'
                f' Currently: {options.docker_entrypoint_sh_setup_overlay}'
            )
        )
        docker_entrypoint_sh_setup_overlay_group.add_argument(
            '--no-docker-entrypoint-sh-setup-overlay',
            action='store_false',
            dest='docker_entrypoint_sh_setup_overlay',
            default=options.docker_entrypoint_sh_setup_overlay,
            help=(
                SUPPRESS if not options.docker_entrypoint_sh_setup_overlay else
                f'Do not source setup.bash from workspace install.'
                f' Currently: {options.docker_entrypoint_sh_setup_overlay}'
            )
        )

        docker_entrypoint_sh_setup_underlay_group = (
            self.docker_entrypoint_sh_setup_underlay.add_mutually_exclusive_group()
        )
        docker_entrypoint_sh_setup_underlay_group.add_argument(
            '--docker-entrypoint-sh-setup-underlay',
            action='store_true',
            default=options.docker_entrypoint_sh_setup_underlay,
            help=(
                f'Source setup.bash from ROS install.'
                f' Currently: {options.docker_entrypoint_sh_setup_underlay}'
            )
        )
        docker_entrypoint_sh_setup_underlay_group.add_argument(
            '--no-docker-entrypoint-sh-setup-underlay',
            action='store_false',
            dest='docker_entrypoint_sh_setup_underlay',
            default=options.docker_entrypoint_sh_setup_underlay,
            help=(
                SUPPRESS if not options.docker_entrypoint_sh_setup_underlay else
                f'Do not source setup.bash from ROS install.'
                f' Currently: {options.docker_entrypoint_sh_setup_underlay}'
            )
        )

        docker_image_pull_group = self.docker_image_pull.add_mutually_exclusive_group()
        docker_image_pull_group.add_argument(
            '--docker-image-pull',
            action='store_true',
            default=options.docker_image_pull,
            help=(
                SUPPRESS if options.docker_image_pull else
                f'Pull newer docker image.'
                f' Currently: {options.docker_image_pull}'
            )
        )
        docker_image_pull_group.add_argument(
            '--no-docker-image-pull',
            action='store_false',
            dest='docker_image_pull',
            default=options.docker_image_pull,
            help=(
                SUPPRESS if not options.docker_image_pull else
                f'Do not pull newer docker image.'
                f' Currently: {options.docker_image_pull}'
            )
        )

        docker_image_replace_group = (
            self.docker_image_replace.add_mutually_exclusive_group()
        )
        docker_image_replace_group.add_argument(
            '--docker-image-replace',
            action='store_true',
            default=options.docker_image_replace,
            help=(
                SUPPRESS if options.docker_image_replace else
                f'Replace docker images that should only need to be built once.'
                f' Currently: {options.docker_image_replace}'
            )
        )
        docker_image_replace_group.add_argument(
            '--no-docker-image-replace',
            action='store_false',
            dest='docker_image_replace',
            default=options.docker_image_replace,
            help=(
                SUPPRESS if not options.docker_image_replace else
                f'Do not replace docker images that should only need to be built once.'
                f' Currently: {options.docker_image_replace}'
            )
        )

        dry_run_group = (
            self.dry_run.add_mutually_exclusive_group()
        )
        dry_run_group.add_argument(
            '--dry-run',
            action='store_true',
            default=options.dry_run,
            help=(
                f'Skip actions that cause lasting effects. Dependant actions to fail.'
                f' Currently: {options.dry_run}'
            )
        )
        dry_run_group.add_argument(
            '--no-dry-run',
            action='store_false',
            dest='dry_run',
            default=options.dry_run,
            help=(
                SUPPRESS if not options.dry_run else
                f'Do not skip actions that cause lasting effects.'
                f' Currently: {options.dry_run}'
            )
        )

        class HelpAction(Action):

            def __call__(
                    self,
                    _parser: ArgumentParser,
                    namespace,
                    values,
                    option_string=None
            ) -> None:
                setattr(namespace, self.dest, _parser.format_help())

        self.help.add_argument(
            '--help', '-h',
            action=HelpAction,
            nargs=0,
            help='show this help message and exit',
        )

        self.idea_ide_name.add_argument(
            '--idea-ide-name',
            default=options.idea_ide_name,
            choices=choices.idea_ide_name,
            help=f'Name of IDEA IDE to use. Currently: {options.idea_ide_name}'
        )

        self.idea_uuid.add_argument(
            '--idea-uuid',
            default=options.idea_uuid,
            type=str,
            help=(
                f'UUID to use for generated idea settings. '
                f' Currently: {options.idea_uuid}'
            )
        )

        self.log_level.add_argument(
            '--log-level',
            default=options.log_level,
            choices=choices.log_level,
            help=f'Currently: {options.log_level}',
        )

        self.release.add_argument(
            '--release', '-r',
            default=options.release,
            choices=choices.release,
            help=f'ROS release to build. Currently: {options.release}',
        )

        reset_cache_group = self.reset_cache.add_mutually_exclusive_group()
        reset_cache_group.add_argument(
            '--reset-cache',
            default=options.reset_cache,
            action='store_true',
            help=(
                SUPPRESS if options.reset_cache else
                f'Reset persistent memoize cache. Currently: {options.reset_cache}'
            )
        )
        reset_cache_group.add_argument(
            '--no-reset-cache',
            dest='reset_cache',
            default=options.reset_cache,
            action='store_false',
            help=(
                SUPPRESS if not options.reset_cache else
                f'Do not reset persistent memoize cache. Currently: {options.reset_cache}'
            )
        )

        run_main_group = self.run_main.add_mutually_exclusive_group()
        run_main_group.add_argument(
            '--run-main',
            default=options.run_main,
            action='store_true',
            help=(
                SUPPRESS if options.run_main else
                f'Run main part of program. Currently: {options.run_main}'
            )
        )
        run_main_group.add_argument(
            '--no-run-main',
            dest='run_main',
            default=options.run_main,
            action='store_false',
            help=(
                SUPPRESS if not options.run_main else
                f'Do not run main part of program. Currently: {options.run_main}'
            )
        )

        run_validate_options_group = self.run_validate_options.add_mutually_exclusive_group()
        run_validate_options_group.add_argument(
            '--run-validate-options',
            default=options.run_validate_options,
            action='store_true',
            help=(
                SUPPRESS if options.run_validate_options else
                f'Run validate options part of program.'
                f' Currently: {options.run_validate_options}'
            )
        )
        run_validate_options_group.add_argument(
            '--skip-validate-options',
            dest='run_validate_options',
            default=options.run_validate_options,
            action='store_false',
            help=(
                SUPPRESS if not options.run_validate_options else
                f'Do not run validate optionspart of program.'
                f' Currently: {options.run_validate_options}'
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
                f'Do not build with a sanitizer enabled.'
                f' Currently: {options.sanitizer}'
            )
        )


def get_flag(options: Options) -> Flag:
    return Flag(options=options)


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

        handler_module = '.'.join([*prev_sub_commands, self.sub_command])
        argument_parser.set_defaults(handler_module=handler_module)

        if not self.sub_parser_by_sub_command:
            handler_class_parts = []

            for sub_command in [*prev_sub_commands[1:], self.sub_command]:
                for sub_command_part in sub_command.split('_'):
                    handler_class_parts.append(sub_command_part.capitalize())
            handler_class = ''.join(handler_class_parts)
            argument_parser.set_defaults(handler_class=handler_class)
        else:
            sub_argument_parser = argument_parser.add_subparsers(required=True)
            for sub_parser in self.sub_parser_by_sub_command.values():
                sub_parser.get_argument_parser(
                    parent_sub_argument_parser=sub_argument_parser,
                    parent_flags=flags,
                    prev_sub_commands=tuple([*prev_sub_commands, self.sub_command])
                )

        return argument_parser

    def parse_args(self, args: Tuple[str]) -> FrozenDict[str, Any]:
        argument_parser = self.get_argument_parser()
        autocomplete(argument_parser)
        # noinspection PyBroadException
        parsed_args = argument_parser.parse_args(args)

        for k, v in parsed_args.__dict__.items():
            if isinstance(v, list):
                setattr(parsed_args, k, frozenset(v))

        parsed_args = frozendict(parsed_args.__dict__)

        return parsed_args

    @staticmethod
    def of_options(options: Optional[Options] = None) -> Parser:
        options = options if options is not None else Options()

        flag = get_flag(options)
        positional = get_positional(options)

        parser = Parser(
            sub_command='rosdev',
            flags=frozenset({
                flag.architecture,
                flag.dry_run,
                flag.help,
                flag.log_level,
                flag.release,
                flag.reset_cache,
                flag.run_main,
                flag.run_validate_options,
            })
        )

        parser = parser.merged_with(
            sub_commands='build',
            positionals=(
                positional.remainder,
            ),
            flags=frozenset({
                flag.backend_ssh_builder_identity_path,
                flag.backend_ssh_builder_uri,
            }),
        )

        parser = parser.merged_with(
            sub_commands='clion',
            flags=frozenset({
                flag.build_type,
                flag.docker_entrypoint_sh_setup_overlay,
                flag.docker_entrypoint_sh_setup_underlay,
                flag.docker_image_pull,
                flag.sanitizer,
            }),
        )

        parser = parser.merged_with(sub_commands='gen architecture')

        parser = parser.merged_with(
            sub_commands='gen backend apt key builder',
            flags=frozenset({
                flag.backend_ssh_builder_identity_path,
                flag.backend_ssh_builder_uri,
            })
        )
        parser = parser.merged_with(
            sub_commands='gen backend apt key runner',
            flags=frozenset({
                flag.backend_ssh_runner_identity_path,
                flag.backend_ssh_runner_uri,
            })
        )
        parser = parser.merged_with(
            sub_commands='gen backend apt packages builder',
            flags=frozenset({
                flag.backend_ssh_builder_identity_path,
                flag.backend_ssh_builder_uri,
            })
        )
        parser = parser.merged_with(
            sub_commands='gen backend apt packages runner',
            flags=frozenset({
                flag.backend_ssh_runner_identity_path,
                flag.backend_ssh_runner_uri,
            })
        )
        parser = parser.merged_with(
            sub_commands='gen backend apt source builder',
            flags=frozenset({
                flag.backend_ssh_builder_identity_path,
                flag.backend_ssh_builder_uri,
            })
        )
        parser = parser.merged_with(
            sub_commands='gen backend apt source runner',
            flags=frozenset({
                flag.backend_ssh_runner_identity_path,
                flag.backend_ssh_runner_uri,
            })
        )
        parser = parser.merged_with(
            sub_commands='gen backend builder',
            flags=frozenset({
                flag.backend_ssh_builder_identity_path,
                flag.backend_ssh_builder_uri,
            })
        )
        parser = parser.merged_with(
            sub_commands='gen backend builder_base',
            flags=frozenset({
                flag.backend_ssh_builder_identity_path,
                flag.backend_ssh_builder_uri,
            })
        )
        parser = parser.merged_with(
            sub_commands='gen backend endpoints',
            flags=frozenset({
                flag.backend_ssh_builder_identity_path,
                flag.backend_ssh_builder_uri,
                flag.backend_ssh_runner_identity_path,
                flag.backend_ssh_runner_uri,
            })
        )
        parser = parser.merged_with(
            sub_commands='gen backend runner',
            flags=frozenset({
                flag.backend_ssh_runner_identity_path,
                flag.backend_ssh_runner_uri,
            })
        )
        parser = parser.merged_with(
            sub_commands='gen backend runner_base',
            flags=frozenset({
                flag.backend_ssh_runner_identity_path,
                flag.backend_ssh_runner_uri,
            })
        )
        parser = parser.merged_with(
            sub_commands='gen backend ssh builder',
            flags=frozenset({
                flag.backend_ssh_builder_identity_path,
                flag.backend_ssh_builder_uri,
            })
        )
        parser = parser.merged_with(
            sub_commands='gen backend ssh builder_base',
            flags=frozenset({
                flag.backend_ssh_builder_identity_path,
                flag.backend_ssh_builder_uri,
            })
        )
        parser = parser.merged_with(
            sub_commands='gen backend ssh runner',
            flags=frozenset({
                flag.backend_ssh_runner_identity_path,
                flag.backend_ssh_runner_uri,
            })
        )
        parser = parser.merged_with(
            sub_commands='gen backend ssh runner_base',
            flags=frozenset({
                flag.backend_ssh_runner_identity_path,
                flag.backend_ssh_runner_uri,
            })
        )

        parser = parser.merged_with(
            sub_commands='gen colcon build',
            flags=frozenset({
                flag.build_type,
                flag.colcon_build_args,
                flag.docker_image_pull,
                flag.sanitizer,
            })
        )

        parser = parser.merged_with(
            sub_commands='gen docker',
            flags=frozenset({
                flag.docker_container_ports,
                flag.docker_container_replace,
                flag.docker_container_volumes,
                flag.docker_entrypoint_sh_setup_overlay,
                flag.docker_entrypoint_sh_setup_underlay,
                flag.docker_image_pull,
                flag.docker_image_replace,
            })
        )
        parser = parser.merged_with(sub_commands='gen docker container')
        parser = parser.merged_with(sub_commands='gen docker container_base')
        parser = parser.merged_with(sub_commands='gen docker dockerfile')
        parser = parser.merged_with(sub_commands='gen docker entrypoint_sh')
        parser = parser.merged_with(sub_commands='gen docker gdbinit')
        parser = parser.merged_with(sub_commands='gen docker image')
        parser = parser.merged_with(sub_commands='gen docker install')
        parser = parser.merged_with(sub_commands='gen docker pam_environment')
        parser = parser.merged_with(sub_commands='gen docker ssh')
        parser = parser.merged_with(sub_commands='gen docker ssh_base')

        parser = parser.merged_with(sub_commands='gen home')
        parser = parser.merged_with(sub_commands='gen host')

        parser = parser.merged_with(
            sub_commands='gen idea',
            flags=frozenset({
                flag.docker_entrypoint_sh_setup_overlay,
                flag.docker_entrypoint_sh_setup_underlay,
                flag.idea_ide_name,
            })
        )

        parser = parser.merged_with(sub_commands='gen idea c_kdbx')
        parser = parser.merged_with(sub_commands='gen idea c_pwd')
        parser = parser.merged_with(sub_commands='gen idea home')
        parser = parser.merged_with(sub_commands='gen idea ide_base')
        parser = parser.merged_with(sub_commands='gen idea security_xml')
        parser = parser.merged_with(sub_commands='gen idea workspace_xml')

        parser = parser.merged_with(sub_commands='gen idea clion cpp_toolchains_xml')
        parser = parser.merged_with(sub_commands='gen idea clion deployment_xml')
        parser = parser.merged_with(sub_commands='gen idea clion iml')
        parser = parser.merged_with(sub_commands='gen idea clion misc_xml')
        parser = parser.merged_with(sub_commands='gen idea clion modules_xml')
        parser = parser.merged_with(sub_commands='gen idea clion webservers_xml')
        parser = parser.merged_with(sub_commands='gen idea clion workspace_xml')

        parser = parser.merged_with(sub_commands='gen idea pycharm deployment_xml')
        parser = parser.merged_with(sub_commands='gen idea pycharm iml')
        parser = parser.merged_with(sub_commands='gen idea pycharm jdk_table_xml')
        parser = parser.merged_with(sub_commands='gen idea pycharm misc_xml')
        parser = parser.merged_with(sub_commands='gen idea pycharm modules_xml')
        parser = parser.merged_with(sub_commands='gen idea pycharm webservers_xml')

        parser = parser.merged_with(
            sub_commands='gen install',
            flags=frozenset({
                flag.docker_image_pull,
            })
        )
        parser = parser.merged_with(sub_commands='gen install_base')

        parser = parser.merged_with(
            sub_commands='gen rosdep install',
            positionals=(
                positional.remainder,
            ),
            flags=frozenset({
                flag.docker_image_pull,
            })
        )
        parser = parser.merged_with(
            sub_commands='gen rosdep update',
            positionals=(
                positional.remainder,
            ),
            flags=frozenset({
                flag.docker_image_pull,
            })
        )

        parser = parser.merged_with(sub_commands='gen rosdev home')
        parser = parser.merged_with(sub_commands='gen rosdev workspace')

        parser = parser.merged_with(
            sub_commands='gen src',
            flags=frozenset({
                flag.docker_image_pull,
            })
        )
        parser = parser.merged_with(sub_commands='gen src_base')

        parser = parser.merged_with(sub_commands='gen workspace')

        parser = parser.merged_with(
            sub_commands='install',
            positionals=(
                positional.remainder,
            ),
        )

        parser = parser.merged_with(
            sub_commands='launch',
            positionals=(
                positional.remainder,
            ),
            flags=frozenset({
                flag.backend_ssh_runner_identity_path,
                flag.backend_ssh_runner_uri,
            })
        )

        parser = parser.merged_with(sub_commands='pycharm')

        parser = parser.merged_with(
            sub_commands='run',
            positionals=(
                positional.remainder,
            ),
            flags=frozenset({
                flag.backend_ssh_runner_identity_path,
                flag.backend_ssh_runner_uri,
            })
        )

        parser = parser.merged_with(
            sub_commands='shell',
            flags=frozenset({
                flag.docker_container_ccache,
                flag.docker_container_gui,
                flag.docker_container_ports,
                flag.docker_container_replace,
                flag.docker_container_volumes,
                flag.docker_entrypoint_sh_setup_overlay,
                flag.docker_entrypoint_sh_setup_underlay,
                flag.docker_image_pull,
                flag.docker_image_replace,
            }),
        )

        return parser
