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
architecture_choices = sorted({architecture_default, 'arm32v7', 'arm64v8'})
architecture_flag = ArgumentParser(add_help=False)
architecture_flag.add_argument(
    '--architecture', '-a',
    default=architecture_default,
    choices=architecture_choices,
    help=f'Architecture to build. Default: {architecture_default}',
)

architectures_default = [architecture_default]
architectures_flag = ArgumentParser(add_help=False)
architectures_flag.add_argument(
    '--architectures', '-a',
    default=[architecture_default],
    nargs='+',
    choices=architecture_choices,
    help=f'List of architectures to build. Default: {architectures_default}',
)

gdbserver_port_default = 1337
gdbserver_port_flag = ArgumentParser(add_help=False)
gdbserver_port_flag.add_argument(
    '--port', '-p',
    dest='gdbserver_port',
    type=int,
    default=1337,
    help=f'Default: {gdbserver_port_default}'
)

log_level_default = 'INFO'
log_level_flag = ArgumentParser(add_help=False)
log_level_flag.add_argument(
    '--log-level',
    default=log_level_default,
    choices=[name for _, name in sorted(logging._levelToName.items())],
    help=f'Default: {log_level_default}',
)

nightly_flag = ArgumentParser(add_help=False)
nightly_flag.add_argument(
    '--nightly', '-n',
    action='store_true',
    help=f'Use osrf/ros2:nightly image as base.'
)

release_default = 'crystal'
release_choices = sorted({'ardent', 'bionic', release_default, 'kinetic', 'melodic'}),
release_flag = ArgumentParser(add_help=False)
release_flag.add_argument(
    '--release', '-r',
    default=release_default,
    choices=release_choices,
    help=f'ROS release to build. Default: {release_default}',
)

releases_default = [release_default]
releases_flag = ArgumentParser(add_help=False)
releases_flag.add_argument(
    '--releases', '-r',
    default=releases_default,
    nargs='+',
    choices=release_choices,
    help=f'List of ROS releases to build. Default: {releases_default}',
)

interactive_flag = ArgumentParser(add_help=False)
interactive_flag.add_argument('--interactive', '-i', action='store_true')

port_flag = ArgumentParser(add_help=False)
port_flag.add_argument('--port', '-p', type=int)

ports_flag = ArgumentParser(add_help=False)
ports_flag.add_argument('--ports', '-p', nargs='*', default=[], type=int)


rosdev_parser = ArgumentParser(parents=[log_level_flag])
rosdev_subparsers = rosdev_parser.add_subparsers(required=True)
rosdev_bash_parser = rosdev_subparsers.add_parser(
    'bash', parents=[architecture_flag, nightly_flag, ports_flag, release_flag])
rosdev_bash_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.bash').handler)
rosdev_clion_parser = rosdev_subparsers.add_parser(
    'clion', parents=[architecture_flag, release_flag])
rosdev_clion_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.clion').handler)
rosdev_gdbserver_parser = rosdev_subparsers.add_parser(
    'gdbserver', parents=[package_positional, gdbserver_port_flag])
rosdev_gdbserver_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gdbserver').handler)
rosdev_gen_parser = rosdev_subparsers.add_parser(
    'gen', parents=[])
rosdev_gen_subparsers = rosdev_gen_parser.add_subparsers(required=True)
rosdev_gen_docker_parser = rosdev_gen_subparsers.add_parser(
    'docker', parents=[])
rosdev_gen_docker_subparsers = rosdev_gen_docker_parser.add_subparsers(required=True)
rosdev_gen_docker_container_parser = rosdev_gen_docker_subparsers.add_parser(
    'container',
    parents=[
        command_positional,
        architecture_flag,
        interactive_flag,
        ports_flag,
        release_flag
    ]
)
rosdev_gen_docker_container_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gen.docker.container').handler)
rosdev_gen_docker_images_parser = rosdev_gen_docker_subparsers.add_parser(
    'images', parents=[architectures_flag, nightly_flag, releases_flag])
rosdev_gen_docker_images_parser.set_defaults(
    get_handler=lambda: import_module('rosdev.gen.docker.images').handler)
