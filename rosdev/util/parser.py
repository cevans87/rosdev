from argparse import ArgumentParser
from dataclasses import dataclass
from importlib import import_module
import logging
import os
import yaml
from rosdev.gen.defaults import Defaults


try:
    with open(f'{os.getcwd()}/.rosdev/defaults', 'r') as defaults_f_in:
        defaults = Defaults(**yaml.load(defaults_f_in.read())['defaults'])
except Exception:
    defaults = Defaults()


@dataclass(frozen=True)
class Positional:
    command = ArgumentParser(add_help=False)
    executable = ArgumentParser(add_help=False)
    package = ArgumentParser(add_help=False)


positional = Positional()
positional.command.add_argument('command')
positional.executable.add_argument('executable')
positional.package.add_argument('package')


@dataclass(frozen=True)
class Flag:
    architecture = ArgumentParser(add_help=False)
    architectures = ArgumentParser(add_help=False)
    asan = ArgumentParser(add_help=False)
    bad_build_num = ArgumentParser(add_help=False)
    bad_release = ArgumentParser(add_help=False)
    build_num = ArgumentParser(add_help=False)
    clean = ArgumentParser(add_help=False)
    colcon_build_args = ArgumentParser(add_help=False)
    debug = ArgumentParser(add_help=False)
    fast = ArgumentParser(add_help=False)
    gdbserver_port = ArgumentParser(add_help=False)
    good_build_num = ArgumentParser(add_help=False)
    good_release = ArgumentParser(add_help=False)
    interactive = ArgumentParser(add_help=False)
    log_level = ArgumentParser(add_help=False)
    port = ArgumentParser(add_help=False)
    ports = ArgumentParser(add_help=False)
    release = ArgumentParser(add_help=False)
    releases = ArgumentParser(add_help=False)


flag = Flag()


@dataclass(frozen=True)
class Choices:
    architecture = sorted({defaults.architecture, 'arm32v7', 'arm64v8'})
    bad_release = sorted({'bionic', 'crystal'}) + ['latest']
    good_release = sorted({'ardent', 'bionic', 'crystal'})
    # noinspection PyProtectedMember
    log_level = [name for _, name in sorted(logging._levelToName.items())]
    release = sorted({'ardent', 'bionic', 'crystal', 'kinetic', 'melodic'}) + [defaults.release]


choices = Choices()

flag.architecture.add_argument(
    '--architecture', '-a',
    default=defaults.architecture,
    choices=choices.architecture,
    help=f'Architecture to build. Default: {defaults.architecture}',
)
flag.architectures.add_argument(
    '--architectures', '-a',
    default=defaults.architectures,
    nargs='+',
    choices=choices.architecture,
    help=f'List of architectures to build. Default: {defaults.architectures}',
)

asan_group = flag.asan.add_mutually_exclusive_group()
asan_group.add_argument(
    '--asan',
    action='store_true',
    help=f'Build with Address Sanitizer enabled. Default: {defaults.asan}'
)
asan_group.add_argument(
    '--no-asan',
    action='store_false',
    dest='asan',
    help=f'Build with Address Sanitizer disabled, Default: {not defaults.asan}'
)

flag.bad_release.add_argument(
    '--bad-release', '-b',
    default=defaults.bad_release,
    choices=choices.bad_release,
    help=f'Bad release to compare. Default: {defaults.bad_release}'
)

flag.bad_build_num.add_argument(
    '--bad-build-num',
    type=int,
    help=f'Bad build number to compare. Supersedes --bad-build'
)

flag.build_num.add_argument(
    '--build-num', '-b',
    type=int,
    help=f'Use specified build from OSRF build farm instead of {defaults.release}'
)

clean_group = flag.clean.add_mutually_exclusive_group()
clean_group.add_argument(
    '--clean',
    action='store_true',
    help=f'Start bash environement without sourcing a ROS setup.bash. Default: {defaults.clean}'
)
clean_group.add_argument(
    '--no-clean',
    action='store_false',
    dest='clean',
    help=f'Start bash environement with sourcing a ROS setup.bash. Default: {not defaults.clean}'
)

flag.colcon_build_args.add_argument(
    '--colcon-build-args',
    help=f'Additional args to pass to colcon build'
)

debug_group = flag.debug.add_mutually_exclusive_group()
debug_group.add_argument(
    '--debug',
    action='store_true',
    help=f'Build with debug enabled. Default: {defaults.debug}'
)
debug_group.add_argument(
    '--no-debug',
    action='store_false',
    dest='debug',
    help=f'Build with debug disabled. Default: {not defaults.debug}'
)

fast_group = flag.fast.add_mutually_exclusive_group()
fast_group.add_argument(
    '--fast', '-f',
    action='store_true',
    help=f'Build from local docker images even when newer ones can be pulled'
)
fast_group.add_argument(
    '--no-fast',
    action='store_false',
    dest='fast',
    help=f'Build from newer docker images when a newer image can be pulled'
)

flag.gdbserver_port.add_argument(
    '--port', '-p',
    type=int,
    default=1337,
    help=f'Default: {defaults.gdbserver_port}'
)
flag.good_build_num.add_argument(
    '--good-build-num',
    type=int,
    help=f'Good build number to compare. Supersedes --good-build'
)

flag.good_release.add_argument(
    '--good-release', '-g',
    default=defaults.good_release,
    choices=choices.good_release,
    help=f'Good release to compare. Default: {defaults.good_release}'
)

interactive_group = flag.interactive.add_mutually_exclusive_group()
interactive_group.add_argument('--interactive', '-i', action='store_true')
interactive_group.add_argument('--no-interactive', dest='interactive', action='store_false')

flag.log_level.add_argument(
    '--log-level',
    default=defaults.log_level,
    choices=choices.log_level,
    help=f'Default: {defaults.log_level}',
)

flag.release.add_argument(
    '--release', '-r',
    default=defaults.release,
    choices=choices.release,
    help=f'ROS release to build. Default: {defaults.release}',
)

flag.releases.add_argument(
    '--releases', '-r',
    default=defaults.releases,
    nargs='+',
    choices=choices.release,
    help=f'List of ROS releases to build. Default: {defaults.releases}',
)

flag.port.add_argument('--port', '-p', type=int)

flag.ports.add_argument('--ports', '-p', nargs='*', default=[], type=int)

rosdev_parser = ArgumentParser(parents=[flag.log_level])
rosdev_subparsers = rosdev_parser.add_subparsers(required=True)
rosdev_bash_parser = rosdev_subparsers.add_parser(
    'bash', parents=[
        flag.architecture,
        flag.build_num,
        flag.clean,
        flag.fast,
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
        flag.build_num, flag.release
    ]
)
rosdev_clion_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.clion').Clion)
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
rosdev_gen_defaults_parser = rosdev_gen_subparsers.add_parser(
    'defaults',
    parents=[
        flag.architecture,
        flag.build_num,
        flag.fast,
        flag.log_level,
        flag.release
    ]
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
        flag.clean,
        flag.fast,
        flag.interactive,
        flag.ports,
        flag.release
    ]
)
rosdev_gen_docker_container_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gen.docker.container').Container)
rosdev_gen_docker_images_parser = rosdev_gen_docker_subparsers.add_parser(
    'images', parents=[flag.architectures, flag.fast, flag.log_level, flag.releases])
rosdev_gen_docker_images_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gen.docker.images').Images)
rosdev_gen_install_parser = rosdev_gen_subparsers.add_parser(
    'install', parents=[flag.architecture, flag.build_num, flag.fast, flag.log_level, flag.release])
rosdev_gen_install_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gen.install').Install)
