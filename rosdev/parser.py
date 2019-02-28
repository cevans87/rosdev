from argparse import ArgumentParser
from importlib import import_module
import logging


command_positional = ArgumentParser(add_help=False)
command_positional.add_argument('command')

executable_positional = ArgumentParser(add_help=False)
executable_positional.add_argument('executable')

package_positional = ArgumentParser(add_help=False)
package_positional.add_argument('package')

architecture_default = 'amd64'
architectures_default = [architecture_default]
bad_build_default = 'latest'
gdbserver_port_default = 1337
good_build_default = 'crystal'
log_level_default = 'INFO'
release_default = 'latest'
releases_default = [release_default]

architecture_choices = sorted({architecture_default, 'arm32v7', 'arm64v8'})
architecture_flag = ArgumentParser(add_help=False)
architecture_flag.add_argument(
    '--architecture', '-a',
    default=architecture_default,
    choices=architecture_choices,
    help=f'Architecture to build. Default: {architecture_default}',
)

architectures_flag = ArgumentParser(add_help=False)
architectures_flag.add_argument(
    '--architectures', '-a',
    default=[architecture_default],
    nargs='+',
    choices=architecture_choices,
    help=f'List of architectures to build. Default: {architectures_default}',
)

asan_flag = ArgumentParser(add_help=False)
asan_flag.add_argument(
    '--asan',
    action='store_true',
    help=f'Build with Address Sanitizer enabled'
)

bad_build_choices = sorted({'bionic', 'crystal'}) + ['latest']
bad_build_flag = ArgumentParser(add_help=False)
bad_build_flag.add_argument(
    '--bad-build', '-b',
    default=bad_build_default,
    choices=bad_build_choices,
    help=f'Bad build to compare. Default: {bad_build_default}'
)

bad_build_num_flag = ArgumentParser(add_help=False)
bad_build_num_flag.add_argument(
    '--bad-build-num',
    type=int,
    help=f'Bad build number to compare. Supersedes --bad-build'
)

build_num_flag = ArgumentParser(add_help=False)
build_num_flag.add_argument(
    '--build-num', '-b',
    type=int,
    help=f'Use specified build from OSRF build farm instead of {release_default}'
)

colcon_build_args_flag = ArgumentParser(add_help=False)
colcon_build_args_flag.add_argument(
    '--colcon-build-args', '-c',
    help=f'Additional args to pass to colcon build'
)

fast_flag = ArgumentParser(add_help=False)
fast_flag.add_argument(
    '--fast', '-f',
    action='store_true',
    help=f'Build from existing local docker images, even if newer ones can be pulled'
)

gdbserver_port_flag = ArgumentParser(add_help=False)
gdbserver_port_flag.add_argument(
    '--port', '-p',
    type=int,
    default=1337,
    help=f'Default: {gdbserver_port_default}'
)

good_build_choices = sorted({'ardent', 'bionic', 'crystal'})
good_build_flag = ArgumentParser(add_help=False)
good_build_flag.add_argument(
    '--good-build', '-g',
    default=good_build_default,
    choices=good_build_choices,
    help=f'Good build to compare. Default: {good_build_default}'
)

good_build_num_flag = ArgumentParser(add_help=False)
good_build_num_flag.add_argument(
    '--good-build-num',
    type=int,
    help=f'Good build number to compare. Supersedes --good-build'
)

interactive_flag = ArgumentParser(add_help=False)
interactive_flag.add_argument('--interactive', '-i', action='store_true')

log_level_flag = ArgumentParser(add_help=False)
# noinspection PyProtectedMember
log_level_flag.add_argument(
    '--log-level',
    default=log_level_default,
    choices=[name for _, name in sorted(logging._levelToName.items())],
    help=f'Default: {log_level_default}',
)

release_choices = sorted({'ardent', 'bionic', 'crystal', 'kinetic', 'melodic'}) + [release_default]
release_flag = ArgumentParser(add_help=False)
release_flag.add_argument(
    '--release', '-r',
    default=release_default,
    choices=release_choices,
    help=f'ROS release to build. Default: {release_default}',
)

releases_flag = ArgumentParser(add_help=False)
releases_flag.add_argument(
    '--releases', '-r',
    default=releases_default,
    nargs='+',
    choices=release_choices,
    help=f'List of ROS releases to build. Default: {releases_default}',
)

port_flag = ArgumentParser(add_help=False)
port_flag.add_argument('--port', '-p', type=int)

ports_flag = ArgumentParser(add_help=False)
ports_flag.add_argument('--ports', '-p', nargs='*', default=[], type=int)

rosdev_parents = [log_level_flag]
rosdev_parser = ArgumentParser(parents=rosdev_parents)
rosdev_subparsers = rosdev_parser.add_subparsers(required=True)
rosdev_bash_parser = rosdev_subparsers.add_parser(
    'bash', parents=[
        architecture_flag,
        build_num_flag,
        fast_flag,
        log_level_flag,
        ports_flag,
        release_flag,
    ]
)
rosdev_bash_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.bash').Bash)
rosdev_bisect_parser = rosdev_subparsers.add_parser(
    'bisect',
    parents=[
        command_positional,
        architecture_flag,
        asan_flag,
        bad_build_flag,
        bad_build_num_flag,
        colcon_build_args_flag,
        fast_flag,
        good_build_flag,
        good_build_num_flag,
        release_flag
    ]
)
rosdev_bisect_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.bisect').Bisect)
rosdev_clion_parser = rosdev_subparsers.add_parser(
    'clion', parents=[architecture_flag, build_num_flag, release_flag])
rosdev_clion_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.clion').Clion)
rosdev_gdbserver_parser = rosdev_subparsers.add_parser(
    'gdbserver', parents=[
        package_positional,
        executable_positional,
        architecture_flag,
        build_num_flag,
        fast_flag,
        gdbserver_port_flag,
        release_flag,
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
        architecture_flag,
        asan_flag,
        build_num_flag,
        colcon_build_args_flag,
        fast_flag,
        release_flag
    ])
rosdev_gen_colcon_build_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gen.colcon.build').Build)
rosdev_gen_docker_parser = rosdev_gen_subparsers.add_parser(
    'docker', parents=[])
rosdev_gen_docker_subparsers = rosdev_gen_docker_parser.add_subparsers(required=True)
rosdev_gen_docker_container_parser = rosdev_gen_docker_subparsers.add_parser(
    'container',
    parents=[
        command_positional,
        architecture_flag,
        fast_flag,
        interactive_flag,
        ports_flag,
        release_flag
    ]
)
rosdev_gen_docker_container_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gen.docker.container').Container)
rosdev_gen_docker_images_parser = rosdev_gen_docker_subparsers.add_parser(
    'images', parents=[architectures_flag, fast_flag, releases_flag])
rosdev_gen_docker_images_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gen.docker.images').Images)
rosdev_gen_install_parser = rosdev_gen_subparsers.add_parser(
    'install', parents=[architecture_flag, build_num_flag, fast_flag, log_level_flag, release_flag])
rosdev_gen_install_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gen.install').Install)
